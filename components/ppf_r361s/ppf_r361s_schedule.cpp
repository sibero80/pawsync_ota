#include "ppf_r361s.h"
#include "esphome/core/log.h"
#include <cmath>
#include <cstring>
#include <string>

namespace esphome {
namespace ppf_r361s {

static const char *const TAG = "ppf_r361s.schedule";

void PPFR361SComponent::load_schedule_from_nvs_() {
  ESPPreferenceObject pref = global_preferences->make_preference<ScheduleStorage>(fnv1_hash("ppf_r361s_sched"), true);
  if (!pref.load(&this->storage_) || this->storage_.magic != 0x50534348) {
    ESP_LOGI(TAG, "Initializing default schedule storage in NVS flash...");
    this->storage_ = ScheduleStorage{};
    this->storage_.magic = 0x50534348;
    this->storage_.active_plan_idx = 0;
    this->storage_.intelligent_feeding = false;
    this->storage_.grams_per_portion = 8.0f;

    // Initialize Plan 1: Default Daily Plan
    snprintf(this->storage_.plans[0].name, sizeof(this->storage_.plans[0].name), "Daily");
    this->storage_.plans[0].enabled = true;
    this->storage_.plans[0].repeat_days = 0x7F; // All days

    // Meal 1: Breakfast at 08:00 AM (2 portions)
    this->storage_.plans[0].meals[0].enabled = true;
    this->storage_.plans[0].meals[0].hour = 8;
    this->storage_.plans[0].meals[0].minute = 0;
    this->storage_.plans[0].meals[0].portions = 2;
    this->storage_.plans[0].meals[0].days = 0x7F;
    this->storage_.plans[0].meals[0].sound = true;

    // Meal 2: Dinner at 18:00 PM (3 portions)
    this->storage_.plans[0].meals[1].enabled = true;
    this->storage_.plans[0].meals[1].hour = 18;
    this->storage_.plans[0].meals[1].minute = 0;
    this->storage_.plans[0].meals[1].portions = 3;
    this->storage_.plans[0].meals[1].days = 0x7F;
    this->storage_.plans[0].meals[1].sound = true;

    this->save_schedule_to_nvs_();
  } else {
    ESP_LOGI(TAG, "Loaded schedule storage from NVS: Active Plan %u, Density %.1fg/portion",
             this->storage_.active_plan_idx + 1, this->storage_.grams_per_portion);
  }

  if (this->intelligent_feeding_switch_ != nullptr) {
    this->intelligent_feeding_switch_->publish_state(this->storage_.intelligent_feeding);
  }
  if (this->grams_per_portion_number_ != nullptr) {
    this->grams_per_portion_number_->publish_state(this->storage_.grams_per_portion);
  }
  if (this->manual_feed_chime_switch_ != nullptr) {
    this->manual_feed_chime_switch_->publish_state(this->storage_.manual_feed_sound);
  }
  if (this->scheduled_feed_chime_switch_ != nullptr) {
    this->scheduled_feed_chime_switch_->publish_state(this->storage_.scheduled_feed_sound);
  }
  if (this->precision_mode_switch_ != nullptr) {
    this->precision_mode_switch_->publish_state(this->storage_.precision_mode);
  }
}

void PPFR361SComponent::save_schedule_to_nvs_() {
  ESPPreferenceObject pref = global_preferences->make_preference<ScheduleStorage>(fnv1_hash("ppf_r361s_sched"), true);
  pref.save(&this->storage_);
  this->update_schedule_sensors_();
}

void PPFR361SComponent::set_active_plan(uint8_t plan_idx) {
  if (plan_idx >= MAX_PLANS) return;
  ESP_LOGI(TAG, "Switching active feeding plan to Plan %u", plan_idx + 1);
  this->storage_.active_plan_idx = plan_idx;
  this->save_schedule_to_nvs_();
  this->sync_schedules_to_coprocessor();
}

void PPFR361SComponent::set_meal_schedule(uint8_t plan_idx, uint8_t meal_idx, bool enabled,
                                         uint8_t hour, uint8_t minute, uint8_t portions, uint8_t days, bool sound) {
  if (plan_idx >= MAX_PLANS || meal_idx >= MAX_MEALS_PER_PLAN) return;
  if (hour > 23) hour = 23;
  if (minute > 59) minute = 59;
  if (portions == 0) portions = 1;
  if (portions > 20) portions = 20;

  ScheduleMeal &m = this->storage_.plans[plan_idx].meals[meal_idx];
  m.enabled = enabled;
  m.hour = hour;
  m.minute = minute;
  m.portions = portions;
  m.days = days;
  m.sound = sound;

  this->storage_.plans[plan_idx].enabled = true;
  this->storage_.plans[plan_idx].repeat_days = 0x7F;

  ESP_LOGI(TAG, "Configured Plan %u Meal %u: %s %02d:%02d (%d portions, days 0x%02X, sound %d)",
           plan_idx + 1, meal_idx + 1, enabled ? "ENABLED" : "DISABLED", hour, minute, portions, days, sound);
  this->save_schedule_to_nvs_();
  if (plan_idx == this->storage_.active_plan_idx) {
    this->sync_schedules_to_coprocessor();
  }
}

uint8_t PPFR361SComponent::parse_days_string(const std::string &str) {
  std::string s = str;
  for (auto &c : s) c = tolower(c);

  if (s.empty() || s == "all" || s == "daily" || s == "everyday") return 0x7F;
  if (s == "weekdays" || s == "workdays") return 0x3E; // Mon-Fri (bit 1..5)
  if (s == "weekends") return 0x41; // Sat-Sun (bit 0 & 6)

  char *endptr = nullptr;
  long num = strtol(s.c_str(), &endptr, 10);
  if (endptr != s.c_str() && num > 0 && num <= 127) {
    return static_cast<uint8_t>(num);
  }

  uint8_t mask = 0;
  if (s.find("su") != std::string::npos) mask |= 0x01;
  if (s.find("mo") != std::string::npos) mask |= 0x02;
  if (s.find("tu") != std::string::npos) mask |= 0x04;
  if (s.find("we") != std::string::npos) mask |= 0x08;
  if (s.find("th") != std::string::npos) mask |= 0x10;
  if (s.find("fr") != std::string::npos) mask |= 0x20;
  if (s.find("sa") != std::string::npos) mask |= 0x40;

  return mask > 0 ? mask : 0x7F;
}

bool PPFR361SComponent::parse_time_string(const std::string &str, uint8_t &hour, uint8_t &minute) {
  int h = 0, m = 0;
  if (sscanf(str.c_str(), "%d:%d", &h, &m) >= 2) {
    if (h >= 0 && h <= 23 && m >= 0 && m <= 59) {
      hour = static_cast<uint8_t>(h);
      minute = static_cast<uint8_t>(m);
      return true;
    }
  }
  return false;
}

void PPFR361SComponent::sync_schedules_to_coprocessor() {
  this->update_schedule_sensors_();
}

void PPFR361SComponent::set_intelligent_feeding(bool enabled) {
  ESP_LOGI(TAG, "Setting Intelligent Feeding (Leftover Subtraction): %s", enabled ? "ON" : "OFF");
  this->storage_.intelligent_feeding = enabled;
  if (this->intelligent_feeding_switch_ != nullptr)
    this->intelligent_feeding_switch_->publish_state(enabled);
  this->save_schedule_to_nvs_();
}

void PPFR361SComponent::set_grams_per_portion(float grams) {
  if (grams > 0.0f) {
    this->storage_.grams_per_portion = grams;
    ESP_LOGI(TAG, "Saved kibble density to NVS: %.2fg/portion", grams);
    this->save_schedule_to_nvs_();

    uint16_t g_int = static_cast<uint16_t>(std::round(grams));
    this->send_frame_(0x22, OPCODE_MEASURING, 0x00, {
      {1, {0x00}},
      {2, {static_cast<uint8_t>(g_int & 0xFF), static_cast<uint8_t>((g_int >> 8) & 0xFF)}}
    });
  }
}

void PPFR361SComponent::clear_plan(uint8_t plan_idx) {
  if (plan_idx >= MAX_PLANS) return;
  for (size_t m = 0; m < MAX_MEALS_PER_PLAN; m++) {
    this->storage_.plans[plan_idx].meals[m].enabled = false;
  }
  this->save_schedule_to_nvs_();
  if (plan_idx == this->storage_.active_plan_idx) {
    this->update_schedule_sensors_();
  }
}

const ScheduleMeal &PPFR361SComponent::get_meal(uint8_t plan_idx, uint8_t meal_idx) const {
  if (plan_idx >= MAX_PLANS) plan_idx = 0;
  if (meal_idx >= MAX_MEALS_PER_PLAN) meal_idx = 0;
  return this->storage_.plans[plan_idx].meals[meal_idx];
}

void PPFR361SComponent::evaluate_schedules_(const ESPTime &now) {
  uint8_t today_mask = 1 << (now.day_of_week - 1);
  uint8_t plan_idx = this->storage_.active_plan_idx;
  if (plan_idx >= MAX_PLANS) plan_idx = 0;

  const auto &plan = this->storage_.plans[plan_idx];
  if (!plan.enabled || !(plan.repeat_days & today_mask)) {
    this->update_schedule_sensors_();
    return;
  }

  for (size_t m = 0; m < MAX_MEALS_PER_PLAN; m++) {
    const auto &meal = plan.meals[m];
    if (!meal.enabled) continue;
    if (!(meal.days & today_mask)) continue;

    if (now.hour == meal.hour && now.minute == meal.minute) {
      uint16_t portions_to_feed = meal.portions;

      // Intelligent Feeding leftover check
      if (this->storage_.intelligent_feeding && this->bowl_weight_sensor_ != nullptr) {
        float bowl_w = this->bowl_weight_sensor_->state;
        if (!std::isnan(bowl_w) && bowl_w > 0.0f) {
          float g_per_portion = this->storage_.grams_per_portion > 0 ? this->storage_.grams_per_portion : 8.0f;
          float target_weight = meal.portions * g_per_portion;
          if (bowl_w >= target_weight) {
            portions_to_feed = 0;
            ESP_LOGI(TAG, "Intelligent feed: Bowl has %.1fg (>= target %.1fg). Skipping meal %zu.",
                     bowl_w, target_weight, m + 1);

            char tbuf[32];
            snprintf(tbuf, sizeof(tbuf), "%04d-%02d-%02d %02d:%02d",
                     now.year, now.month, now.day_of_month, now.hour, now.minute);
            strncpy(this->tank_storage_.last_adjusted_meal_time_str, tbuf, sizeof(this->tank_storage_.last_adjusted_meal_time_str));
            this->tank_storage_.last_adjusted_meal_epoch = now.timestamp;

            snprintf(this->tank_storage_.last_adjusted_meal_notes,
                     sizeof(this->tank_storage_.last_adjusted_meal_notes),
                     "Meal %zu (%02u:%02u): Skipped (Bowl has %.0fg >= target %.0fg)",
                     m + 1, meal.hour, meal.minute, bowl_w, target_weight);
            this->save_tank_storage_to_nvs_();
            this->publish_tank_stats_();
          } else {
            float needed = target_weight - bowl_w;
            portions_to_feed = static_cast<uint16_t>(std::ceil(needed / g_per_portion));
            if (portions_to_feed > meal.portions) portions_to_feed = meal.portions;
            ESP_LOGI(TAG, "Intelligent feed: Bowl has %.1fg. Adjusted meal %zu portions from %d to %d (target %.1fg)",
                     bowl_w, m + 1, meal.portions, portions_to_feed, target_weight);

            if (portions_to_feed != meal.portions) {
              char tbuf[32];
              snprintf(tbuf, sizeof(tbuf), "%04d-%02d-%02d %02d:%02d",
                       now.year, now.month, now.day_of_month, now.hour, now.minute);
              strncpy(this->tank_storage_.last_adjusted_meal_time_str, tbuf, sizeof(this->tank_storage_.last_adjusted_meal_time_str));
              this->tank_storage_.last_adjusted_meal_epoch = now.timestamp;

              snprintf(this->tank_storage_.last_adjusted_meal_notes,
                       sizeof(this->tank_storage_.last_adjusted_meal_notes),
                       "Meal %zu (%02u:%02u): Reduced from %dp to %dp (-%.0fg)",
                       m + 1, meal.hour, meal.minute, meal.portions, portions_to_feed, bowl_w);
              this->save_tank_storage_to_nvs_();
              this->publish_tank_stats_();
            }
          }
        }
      }

      if (portions_to_feed > 0) {
        ESP_LOGI(TAG, "Executing on-device scheduled meal %zu (Plan %u): %d portions", m + 1, plan_idx + 1, portions_to_feed);
        this->feed(portions_to_feed, meal.sound);
      }
    }
  }

  this->update_schedule_sensors_();
}

void PPFR361SComponent::update_schedule_sensors_() {
  if (this->time_ == nullptr) return;
  ESPTime now = this->time_->now();
  if (!now.is_valid()) return;

  uint8_t today_mask = 1 << (now.day_of_week - 1);
  uint8_t plan_idx = this->storage_.active_plan_idx;
  if (plan_idx >= MAX_PLANS) plan_idx = 0;
  const auto &plan = this->storage_.plans[plan_idx];

  int next_meal_idx = -1;
  int min_diff = 1440 * 2;
  int now_mins = now.hour * 60 + now.minute;

  std::string summary = "";

  for (size_t m = 0; m < MAX_MEALS_PER_PLAN; m++) {
    const auto &meal = plan.meals[m];
    if (!meal.enabled || !(meal.days & today_mask)) continue;

    int meal_mins = meal.hour * 60 + meal.minute;
    int diff = meal_mins - now_mins;
    if (diff > 0 && diff < min_diff) {
      min_diff = diff;
      next_meal_idx = m;
    }

    if (!summary.empty()) summary += " | ";
    char m_buf[32];
    snprintf(m_buf, sizeof(m_buf), "M%zu: %02d:%02d (%dp)", m + 1, meal.hour, meal.minute, meal.portions);
    summary += m_buf;
  }

  if (summary.empty()) {
    summary = "No active meals today";
  }

  if (this->schedule_summary_sensor_ != nullptr) {
    this->schedule_summary_sensor_->publish_state(summary);
  }

  if (this->next_meal_sensor_ != nullptr) {
    if (next_meal_idx >= 0) {
      char buf[32];
      snprintf(buf, sizeof(buf), "%02d:%02d (%d portions)",
               plan.meals[next_meal_idx].hour, plan.meals[next_meal_idx].minute, plan.meals[next_meal_idx].portions);
      this->next_meal_sensor_->publish_state(buf);
    } else {
      this->next_meal_sensor_->publish_state("Done for today");
    }
  }
}

}  // namespace ppf_r361s
}  // namespace esphome
