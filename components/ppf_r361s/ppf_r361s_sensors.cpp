#include "ppf_r361s.h"
#include "esphome/core/log.h"
#include "esphome/core/hal.h"
#include <cmath>
#include <cstring>
#include <algorithm>

#if defined(USE_ESP_IDF)
#include "esp_ota_ops.h"
#include "esp_partition.h"
#include "esp_system.h"
#endif

namespace esphome {
namespace ppf_r361s {

static const char *const TAG = "ppf_r361s.sensors";

void PPFR361SComponent::reset_desiccant() {
  ESP_LOGI(TAG, "Resetting 30-day silica desiccant replacement timer...");
  if (this->time_ != nullptr && this->time_->now().is_valid()) {
    this->storage_.last_desiccant_reset_epoch = this->time_->now().timestamp;
  } else {
    this->storage_.last_desiccant_reset_epoch = static_cast<uint32_t>(::time(nullptr));
  }
  this->save_schedule_to_nvs_();
  if (this->desiccant_days_sensor_ != nullptr)
    this->desiccant_days_sensor_->publish_state(30.0f);
}

void PPFR361SComponent::reset_cleaning() {
  ESP_LOGI(TAG, "Resetting 30-day chute cleaning maintenance timer...");
  if (this->time_ != nullptr && this->time_->now().is_valid()) {
    this->storage_.last_cleaning_reset_epoch = this->time_->now().timestamp;
  } else {
    this->storage_.last_cleaning_reset_epoch = static_cast<uint32_t>(::time(nullptr));
  }
  this->save_schedule_to_nvs_();
  if (this->cleaning_days_sensor_ != nullptr)
    this->cleaning_days_sensor_->publish_state(30.0f);
  if (this->cleaning_due_sensor_ != nullptr)
    this->cleaning_due_sensor_->publish_state(false);
}

void PPFR361SComponent::set_manual_feed_sound(bool sound) {
  this->storage_.manual_feed_sound = sound;
  this->save_schedule_to_nvs_();
  if (this->manual_feed_chime_switch_ != nullptr)
    this->manual_feed_chime_switch_->publish_state(sound);
}

void PPFR361SComponent::set_scheduled_feed_sound(bool sound) {
  this->storage_.scheduled_feed_sound = sound;
  this->save_schedule_to_nvs_();
  if (this->scheduled_feed_chime_switch_ != nullptr)
    this->scheduled_feed_chime_switch_->publish_state(sound);
}

void PPFR361SComponent::set_precision_mode(bool precision) {
  this->storage_.precision_mode = precision;
  this->save_schedule_to_nvs_();
  if (this->precision_mode_switch_ != nullptr)
    this->precision_mode_switch_->publish_state(precision);
}

void PPFR361SComponent::start_portion_calibration(uint16_t portions) {
  if (portions == 0) portions = 2;
  if (portions > 10) portions = 10;

  ESP_LOGI(TAG, "Starting Automated Kibble Portion Calibration wizard (%d portions)...", portions);
  this->calib_portions_ = portions;
  this->calib_state_ = CALIB_TARING;
  this->calib_timer_ = millis();

  if (this->status_text_sensor_ != nullptr)
    this->status_text_sensor_->publish_state("Calibrating: Taring bowl scale...");

  this->tare_bowl();
}

void PPFR361SComponent::set_use_stock_calibration(bool use_stock) {
  this->use_stock_calibration_ = use_stock;
  ESP_LOGI(TAG, "Calibration mode switched to: %s", use_stock ? "Stock Coprocessor 6-Portion Cycle" : "Short Automated 2-Portion Cycle");
  ESPPreferenceObject cal_pref = global_preferences->make_preference<bool>(fnv1_hash("ppf_cal_mode"), true);
  cal_pref.save(&use_stock);
  if (this->stock_calibration_switch_ != nullptr)
    this->stock_calibration_switch_->publish_state(use_stock);
}

void PPFR361SComponent::start_stock_measuring() {
  ESP_LOGI(TAG, "Triggering coprocessor stock measuring cycle (Opcode 0x5599: 6-portions)...");
  this->calib_state_ = CALIB_STOCK_MEASURING;
  this->calib_timer_ = millis();

  if (this->status_text_sensor_ != nullptr)
    this->status_text_sensor_->publish_state("Stock Calibrating (6 portions)...");

  this->send_frame_(0x22, OPCODE_MEASURING, 0x00, {
    {1, {0x01}}, // Key 1: start calibration cycle
    {2, {0x00, 0x00}}
  });
}

void PPFR361SComponent::load_meal_storage_from_nvs_() {
  ESPPreferenceObject pref = global_preferences->make_preference<MealSessionStorage>(fnv1_hash("ppf_r361s_meal"), true);
  if (!pref.load(&this->meal_storage_) || this->meal_storage_.magic != 0x4D45414C) {
    this->meal_storage_ = MealSessionStorage{};
    this->meal_storage_.magic = 0x4D45414C;
    this->save_meal_storage_to_nvs_();
  } else {
    if (this->meal_storage_.total_eaten_today_g == 0.0f && this->meal_storage_.last_meal_consumed_g > 0.0f) {
      this->meal_storage_.total_eaten_today_g = this->meal_storage_.last_meal_consumed_g;
      this->save_meal_storage_to_nvs_();
    }
    ESP_LOGI(TAG, "Loaded meal storage from NVS: Last consumed %.1fg, duration %us, epoch %u, eaten today %.1fg",
             this->meal_storage_.last_meal_consumed_g,
             this->meal_storage_.last_meal_duration_s,
             (unsigned)this->meal_storage_.last_meal_epoch,
             this->meal_storage_.total_eaten_today_g);
    this->publish_meal_stats_();
  }
}

void PPFR361SComponent::save_meal_storage_to_nvs_() {
  ESPPreferenceObject pref = global_preferences->make_preference<MealSessionStorage>(fnv1_hash("ppf_r361s_meal"), true);
  pref.save(&this->meal_storage_);
}

void PPFR361SComponent::process_bowl_weight_sample_(float current_weight) {
  if (this->is_dispensing_ || this->calib_state_ != CALIB_IDLE) return;
  if (std::isnan(current_weight)) return;

  uint32_t now_ms = millis();

  if (this->feed_just_finished_) {
    this->feed_just_finished_ = false;
    this->baseline_bowl_weight_ = current_weight;
    this->drift_candidate_weight_ = current_weight;
    this->refill_candidate_weight_ = 0.0f;
    ESP_LOGI(TAG, "Post-dispense baseline locked at %.1fg", this->baseline_bowl_weight_);
    return;
  }

  if (this->baseline_bowl_weight_ <= 0.0f && current_weight > 0.0f) {
    this->baseline_bowl_weight_ = current_weight;
    this->drift_candidate_weight_ = current_weight;
    ESP_LOGD(TAG, "Bowl baseline weight established: %.1fg", this->baseline_bowl_weight_);
    return;
  }

  if (current_weight <= 0.0f) {
    if (this->eating_state_ == EATING_IN_PROGRESS) {
      ESP_LOGW(TAG, "Bowl removed during eating session. Cancelling eating detection.");
      this->eating_state_ = EATING_IDLE;
      if (this->pet_eating_sensor_ != nullptr)
        this->pet_eating_sensor_->publish_state(false);
      this->poll_interval_ms_ = this->ac_power_ ? 30000 : 120000;
    }
    return;
  }

  if (this->eating_state_ == EATING_IDLE) {
    if (current_weight > this->baseline_bowl_weight_ + 2.0f) {
      if (std::abs(current_weight - this->refill_candidate_weight_) <= 1.0f) {
        if (now_ms - this->refill_candidate_start_ms_ >= 3000) {
          ESP_LOGI(TAG, "Confirmed food addition (%.1fg -> %.1fg), updating baseline.",
                   this->baseline_bowl_weight_, current_weight);
          this->baseline_bowl_weight_ = current_weight;
          this->drift_candidate_weight_ = current_weight;
          this->refill_candidate_weight_ = 0.0f;
        }
      } else {
        this->refill_candidate_weight_ = current_weight;
        this->refill_candidate_start_ms_ = now_ms;
      }
      return;
    } else {
      this->refill_candidate_weight_ = 0.0f;
    }

    float drop_from_baseline = this->baseline_bowl_weight_ - current_weight;
    if (drop_from_baseline >= 1.5f && this->baseline_bowl_weight_ >= 2.0f) {
      ESP_LOGI(TAG, "Pet eating session detected! Baseline: %.1fg, Current: %.1fg, Drop: %.1fg",
               this->baseline_bowl_weight_, current_weight, drop_from_baseline);
      this->eating_state_ = EATING_IN_PROGRESS;
      this->eating_start_weight_ = this->baseline_bowl_weight_;
      this->eating_last_weight_ = current_weight;
      this->eating_start_ms_ = now_ms;
      this->eating_last_change_ms_ = now_ms;
      this->poll_interval_ms_ = 5000;

      if (this->pet_eating_sensor_ != nullptr)
        this->pet_eating_sensor_->publish_state(true);
    } else if (std::abs(current_weight - this->baseline_bowl_weight_) <= 1.0f && current_weight > 0.0f) {
      if (std::abs(current_weight - this->drift_candidate_weight_) <= 0.5f) {
        if (now_ms - this->drift_candidate_start_ms_ >= 180000) {
          if (current_weight != this->baseline_bowl_weight_) {
            ESP_LOGD(TAG, "Slow ambient load-cell drift adapted: baseline %.1fg -> %.1fg",
                     this->baseline_bowl_weight_, current_weight);
            this->baseline_bowl_weight_ = current_weight;
          }
          this->drift_candidate_start_ms_ = now_ms;
        }
      } else {
        this->drift_candidate_weight_ = current_weight;
        this->drift_candidate_start_ms_ = now_ms;
      }
    }
  } else if (this->eating_state_ == EATING_IN_PROGRESS) {
    if (current_weight < this->eating_last_weight_ || std::abs(current_weight - this->eating_last_weight_) >= 1.0f) {
      ESP_LOGD(TAG, "Eating in progress: weight changed %.1fg -> %.1fg", this->eating_last_weight_, current_weight);
      this->eating_last_change_ms_ = now_ms;
      this->eating_last_weight_ = current_weight;
    }
  }
}

void PPFR361SComponent::check_eating_settle_timeout_() {
  if (this->eating_state_ != EATING_IN_PROGRESS) return;

  uint32_t now_ms = millis();
  if (now_ms - this->eating_last_change_ms_ >= 60000) {
    float consumed = this->eating_start_weight_ - this->eating_last_weight_;
    uint32_t duration_s = (this->eating_last_change_ms_ - this->eating_start_ms_) / 1000;
    if (duration_s == 0) duration_s = 1;

    ESP_LOGI(TAG, "Eating session settled after 60s stability window. Start: %.1fg, End: %.1fg, Consumed: %.1fg, Duration: %us",
             this->eating_start_weight_, this->eating_last_weight_, consumed, (unsigned)duration_s);

    if (consumed >= 2.0f) {
      this->meal_storage_.last_meal_consumed_g = consumed;
      this->meal_storage_.last_meal_duration_s = duration_s;
      this->meal_storage_.last_meal_epoch = this->time_ != nullptr ? this->time_->now().timestamp : 0;
      this->meal_storage_.total_eaten_today_g += consumed;
      if (this->time_ != nullptr) {
        this->meal_storage_.last_day_of_year = this->time_->now().day_of_year;
      }

      float speed_g_per_min = (consumed / static_cast<float>(duration_s)) * 60.0f;
      this->meal_storage_.last_eating_speed_g_per_min = speed_g_per_min;
      if (this->eating_speed_sensor_ != nullptr) {
        this->eating_speed_sensor_->publish_state(speed_g_per_min);
      }

      if (this->abnormal_eating_sensor_ != nullptr) {
        bool abnormal = (speed_g_per_min > 45.0f || speed_g_per_min < 1.5f);
        this->abnormal_eating_sensor_->publish_state(abnormal);
      }

      this->save_meal_storage_to_nvs_();
      this->publish_meal_stats_();

      this->meal_eaten_callback_.call(consumed, duration_s, speed_g_per_min);
    } else {
      ESP_LOGI(TAG, "Consumed weight %.1fg < 2.0g threshold. Discarding noise / sniffing event.", consumed);
    }

    this->baseline_bowl_weight_ = this->eating_last_weight_;
    this->drift_candidate_weight_ = this->eating_last_weight_;
    this->refill_candidate_weight_ = 0.0f;
    this->poll_interval_ms_ = this->ac_power_ ? 30000 : 120000;
    this->eating_state_ = EATING_IDLE;

    if (this->pet_eating_sensor_ != nullptr)
      this->pet_eating_sensor_->publish_state(false);
  }
}

void PPFR361SComponent::publish_meal_stats_() {
  if (this->last_meal_consumed_sensor_ != nullptr && this->meal_storage_.last_meal_consumed_g > 0.0f) {
    this->last_meal_consumed_sensor_->publish_state(this->meal_storage_.last_meal_consumed_g);
  }
  if (this->last_meal_duration_sensor_ != nullptr && this->meal_storage_.last_meal_duration_s > 0) {
    this->last_meal_duration_sensor_->publish_state(this->meal_storage_.last_meal_duration_s);
  }
  if (this->eating_speed_sensor_ != nullptr && this->meal_storage_.last_eating_speed_g_per_min > 0.0f) {
    this->eating_speed_sensor_->publish_state(this->meal_storage_.last_eating_speed_g_per_min);
  }
  if (this->total_eaten_today_sensor_ != nullptr) {
    this->total_eaten_today_sensor_->publish_state(this->meal_storage_.total_eaten_today_g);
  }
  if (this->last_meal_time_sensor_ != nullptr && this->meal_storage_.last_meal_epoch > 0) {
    if (this->time_ != nullptr) {
      ESPTime t = ESPTime::from_epoch_local(this->meal_storage_.last_meal_epoch);
      if (t.is_valid()) {
        char buf[32];
        t.strftime(buf, sizeof(buf), "%Y-%m-%d %H:%M");
        this->last_meal_time_sensor_->publish_state(buf);
      }
    }
  }
}

void PPFR361SComponent::set_stock_partition_state(uint8_t state) {
  this->stock_partition_state_ = state;
  this->is_stock_intact_ = (state == STATE_STOCK_INTACT);
  ESPPreferenceObject stock_pref = global_preferences->make_preference<uint8_t>(fnv1_hash("ppf_stock_state"), true);
  stock_pref.save(&state);
}

void PPFR361SComponent::rollback_to_stock_firmware() {
  if (this->stock_partition_state_ == STATE_STOCK_OVERWRITTEN || !this->is_stock_intact_) {
    ESP_LOGE(TAG, "Cannot rollback: alternate partition is NOT factory stock firmware (already overwritten by ESPHome). Use 1-Click Restore service in Home Assistant.");
    return;
  }
  ESP_LOGW(TAG, "Initiating software rollback to alternate partition...");
#if defined(USE_ESP_IDF)
  const esp_partition_t *running = esp_ota_get_running_partition();
  const char *target_label = (running != nullptr && strcmp(running->label, "ota_1") == 0) ? "ota_0" : "ota_1";
  
  const esp_partition_t *target = esp_partition_find_first(ESP_PARTITION_TYPE_APP, ESP_PARTITION_SUBTYPE_ANY, target_label);
  if (target != nullptr) {
    esp_err_t err = esp_ota_set_boot_partition(target);
    if (err == ESP_OK) {
      ESP_LOGW(TAG, "Boot partition set to '%s' successfully! Rebooting into alternate firmware in 2 seconds...", target_label);
      delay(2000);
      esp_restart();
    } else {
      ESP_LOGE(TAG, "Failed to set boot partition to '%s': 0x%x", target_label, err);
    }
  } else {
    ESP_LOGE(TAG, "Target partition '%s' not found in partition table!", target_label);
  }
#else
  ESP_LOGE(TAG, "Rollback to stock firmware is only supported under ESP-IDF framework.");
#endif
}

void PPFR361SComponent::load_tank_storage_from_nvs_() {
  ESPPreferenceObject pref = global_preferences->make_preference<TankStorage>(fnv1_hash("ppf_r361s_tank"), true);
  if (!pref.load(&this->tank_storage_) || this->tank_storage_.magic != 0x54414E4B) {
    ESP_LOGI(TAG, "Initializing default tank storage in NVS flash...");
    this->tank_storage_ = TankStorage{};
    this->tank_storage_.magic = 0x54414E4B;
    this->tank_storage_.initial_tank_weight_g = 1600.0f;
    this->tank_storage_.current_food_in_tank_g = 1600.0f;
    this->tank_storage_.tank_calibration_volume_l = 4.0f;
    this->tank_storage_.tank_calibration_weight_g = 0.0f;
    this->tank_storage_.reserve_constant_g = 150.0f;
    this->save_tank_storage_to_nvs_();
  } else {
    ESP_LOGI(TAG, "Loaded tank storage from NVS: Food remaining %.1fg / %.1fg (%.1fL)",
             this->tank_storage_.current_food_in_tank_g,
             this->tank_storage_.initial_tank_weight_g,
             this->tank_storage_.tank_calibration_volume_l);
  }
}

void PPFR361SComponent::save_tank_storage_to_nvs_() {
  ESPPreferenceObject pref = global_preferences->make_preference<TankStorage>(fnv1_hash("ppf_r361s_tank"), true);
  pref.save(&this->tank_storage_);
}

void PPFR361SComponent::publish_tank_stats_() {
  float current_g = this->tank_storage_.current_food_in_tank_g;
  if (current_g < 0.0f) current_g = 0.0f;
  float initial_g = this->tank_storage_.initial_tank_weight_g;
  if (initial_g < 100.0f) initial_g = 1600.0f;

  float level_pct = (current_g / initial_g) * 100.0f;
  if (level_pct > 100.0f) level_pct = 100.0f;
  if (level_pct < 0.0f) level_pct = 0.0f;

  if (this->food_in_tank_sensor_ != nullptr)
    this->food_in_tank_sensor_->publish_state(current_g);
  if (this->food_tank_level_sensor_ != nullptr)
    this->food_tank_level_sensor_->publish_state(level_pct);
  if (this->food_empty_sensor_ != nullptr)
    this->food_empty_sensor_->publish_state(current_g <= 0.0f);
  if (this->food_tank_category_sensor_ != nullptr) {
    if (current_g <= 0.0f) {
      this->food_tank_category_sensor_->publish_state("Empty");
    } else if (level_pct <= 30.0f) {
      this->food_tank_category_sensor_->publish_state("Low");
    } else if (level_pct <= 70.0f) {
      this->food_tank_category_sensor_->publish_state("Medium");
    } else {
      this->food_tank_category_sensor_->publish_state("Full");
    }
  }
  if (this->last_refill_time_sensor_ != nullptr && this->tank_storage_.last_refill_epoch > 0) {
    ESPTime t = ESPTime::from_epoch_local(this->tank_storage_.last_refill_epoch);
    if (t.is_valid()) {
      char rbuf[32];
      t.strftime(rbuf, sizeof(rbuf), "%Y-%m-%d %H:%M");
      this->last_refill_time_sensor_->publish_state(rbuf);
    }
  }
  if (this->tank_refill_volume_number_ != nullptr)
    this->tank_refill_volume_number_->publish_state(this->tank_storage_.tank_calibration_volume_l);
  if (this->tank_refill_weight_number_ != nullptr)
    this->tank_refill_weight_number_->publish_state(this->tank_storage_.tank_calibration_weight_g);
  if (this->tank_projection_mode_switch_ != nullptr)
    this->tank_projection_mode_switch_->publish_state(this->tank_storage_.projection_use_rolling_avg);

  if (this->last_adjusted_meal_sensor_ != nullptr)
    this->last_adjusted_meal_sensor_->publish_state(this->tank_storage_.last_adjusted_meal_time_str);
  if (this->last_adjusted_meal_notes_sensor_ != nullptr)
    this->last_adjusted_meal_notes_sensor_->publish_state(this->tank_storage_.last_adjusted_meal_notes);

  this->calculate_and_publish_days_to_empty_();
}

void PPFR361SComponent::calculate_and_publish_days_to_empty_() {
  if (this->days_to_empty_tank_sensor_ == nullptr) return;

  float current_g = this->tank_storage_.current_food_in_tank_g;
  if (current_g <= 0.0f) {
    this->days_to_empty_tank_sensor_->publish_state(0.0f);
    return;
  }

  float daily_burn_g = 0.0f;

  if (this->tank_storage_.projection_use_rolling_avg) {
    float sum = 0.0f;
    int count = 0;
    for (int i = 0; i < 3; i++) {
      if (this->tank_storage_.past_days_consumed_g[i] > 1.0f) {
        sum += this->tank_storage_.past_days_consumed_g[i];
        count++;
      }
    }
    if (count > 0) {
      daily_burn_g = sum / static_cast<float>(count);
    } else if (this->meal_storage_.total_eaten_today_g > 1.0f) {
      daily_burn_g = this->meal_storage_.total_eaten_today_g;
    }
  }

  if (daily_burn_g <= 1.0f) {
    uint8_t plan_idx = this->storage_.active_plan_idx;
    if (plan_idx >= MAX_PLANS) plan_idx = 0;
    const auto &plan = this->storage_.plans[plan_idx];
    uint16_t planned_portions = 0;
    if (plan.enabled) {
      for (size_t m = 0; m < MAX_MEALS_PER_PLAN; m++) {
        if (plan.meals[m].enabled) {
          planned_portions += plan.meals[m].portions;
        }
      }
    }
    float g_per_portion = this->storage_.grams_per_portion > 0 ? this->storage_.grams_per_portion : 8.0f;
    daily_burn_g = planned_portions * g_per_portion;
  }

  if (daily_burn_g > 1.0f) {
    float days = current_g / daily_burn_g;
    this->days_to_empty_tank_sensor_->publish_state(days);
  } else {
    this->days_to_empty_tank_sensor_->publish_state(NAN);
  }
}

void PPFR361SComponent::reset_tank_food() {
  ESP_LOGI(TAG, "Tank refill button triggered! Recalibrating tank food capacity...");
  float initial_g = 0.0f;

  if (this->tank_storage_.tank_calibration_weight_g > 0.0f) {
    initial_g = this->tank_storage_.tank_calibration_weight_g;
    ESP_LOGI(TAG, "Refill set via explicit weight entry: %.1fg", initial_g);
  } else {
    float vol_l = this->tank_storage_.tank_calibration_volume_l;
    if (vol_l <= 0.0f) vol_l = 4.0f;
    float g_per_portion = this->storage_.grams_per_portion > 0 ? this->storage_.grams_per_portion : 8.0f;
    float bulk_density = g_per_portion / 18.0f;
    initial_g = vol_l * 1000.0f * bulk_density;
    ESP_LOGI(TAG, "Refill derived from volume (%.1fL * %.3fg/ml): %.1fg", vol_l, bulk_density, initial_g);
  }

  this->tank_storage_.initial_tank_weight_g = initial_g;
  this->tank_storage_.current_food_in_tank_g = initial_g;
  if (this->time_ != nullptr && this->time_->now().is_valid()) {
    this->tank_storage_.last_refill_epoch = this->time_->now().timestamp;
  }
  this->save_tank_storage_to_nvs_();
  this->publish_tank_stats_();
}

void PPFR361SComponent::set_tank_refill_volume(float volume_l) {
  if (volume_l > 0.0f && volume_l <= 10.0f) {
    ESP_LOGI(TAG, "Setting tank refill volume: %.1f L", volume_l);
    this->tank_storage_.tank_calibration_volume_l = volume_l;
    if (this->tank_refill_volume_number_ != nullptr)
      this->tank_refill_volume_number_->publish_state(volume_l);
    this->save_tank_storage_to_nvs_();
  }
}

void PPFR361SComponent::set_tank_refill_weight(float weight_g) {
  if (weight_g >= 0.0f) {
    ESP_LOGI(TAG, "Setting tank refill explicit weight: %.1f g", weight_g);
    this->tank_storage_.tank_calibration_weight_g = weight_g;
    if (this->tank_refill_weight_number_ != nullptr)
      this->tank_refill_weight_number_->publish_state(weight_g);
    this->save_tank_storage_to_nvs_();
  }
}

void PPFR361SComponent::set_tank_projection_mode(bool use_rolling_avg) {
  ESP_LOGI(TAG, "Setting tank projection mode: %s", use_rolling_avg ? "3-Day Rolling Average" : "Scheduled Active Plan");
  this->tank_storage_.projection_use_rolling_avg = use_rolling_avg;
  if (this->tank_projection_mode_switch_ != nullptr)
    this->tank_projection_mode_switch_->publish_state(use_rolling_avg);
  this->save_tank_storage_to_nvs_();
  this->calculate_and_publish_days_to_empty_();
}

}  // namespace ppf_r361s
}  // namespace esphome
