#include "ppf_r361s.h"
#include "esphome/core/log.h"
#include "esphome/core/hal.h"
#include <algorithm>
#include <cmath>
#include <vector>

namespace esphome {
namespace ppf_r361s {

static const char *const TAG = "ppf_r361s";

PPFR361SComponent::PPFR361SComponent() {
  this->backup_server_ = new PPFR361SBackupServer(this);
}

PPFR361SComponent::~PPFR361SComponent() {
  if (this->backup_server_ != nullptr) {
    delete this->backup_server_;
    this->backup_server_ = nullptr;
  }
}

void PPFR361SComponent::setup() {
  ESP_LOGCONFIG(TAG, "Setting up Pawsync PPF-R361S Dual-MCU UART Bridge...");
  this->rx_buffer_.reserve(256);

  // Load persistent schedules from NVS
  this->load_schedule_from_nvs_();

  // Load persistent meal consumption statistics from NVS
  this->load_meal_storage_from_nvs_();

  // Load persistent tank food accounting from NVS
  this->load_tank_storage_from_nvs_();

  // Start lightweight factory backup HTTP streamer
  this->start_backup_http_server();

  // Indicate solid connected Wi-Fi state to coprocessor front LED
  this->set_wifi_led(2);

  // Ensure coprocessor is unlocked on startup so remote & manual feed work out-of-the-box
  this->set_child_lock(false);

  // Push active schedule to coprocessor EEPROM for autonomous offline execution
  this->sync_schedules_to_coprocessor();

  // Request initial status report from coprocessor
  this->poll_status_();

  // Initialize baseline sensor states so Home Assistant entities are immediately populated
  if (this->battery_level_sensor_ != nullptr)
    this->battery_level_sensor_->publish_state(NAN);
  if (this->battery_installed_sensor_ != nullptr)
    this->battery_installed_sensor_->publish_state(false);
  if (this->battery_low_sensor_ != nullptr)
    this->battery_low_sensor_->publish_state(false);
  if (this->bowl_weight_sensor_ != nullptr)
    this->bowl_weight_sensor_->publish_state(0.0f);
  if (this->power_adapter_sensor_ != nullptr)
    this->power_adapter_sensor_->publish_state(true);
  if (this->food_low_sensor_ != nullptr)
    this->food_low_sensor_->publish_state(false);
  if (this->food_empty_sensor_ != nullptr)
    this->food_empty_sensor_->publish_state(false);
  if (this->motor_jammed_sensor_ != nullptr)
    this->motor_jammed_sensor_->publish_state(false);
  if (this->bowl_overflow_sensor_ != nullptr)
    this->bowl_overflow_sensor_->publish_state(false);
  if (this->scale_overload_sensor_ != nullptr)
    this->scale_overload_sensor_->publish_state(false);
  if (this->bowl_connected_sensor_ != nullptr)
    this->bowl_connected_sensor_->publish_state(true);
  if (this->scale_tray_connected_sensor_ != nullptr)
    this->scale_tray_connected_sensor_->publish_state(true);
  if (this->bowl_present_sensor_ != nullptr)
    this->bowl_present_sensor_->publish_state(true);
  if (this->pet_eating_sensor_ != nullptr)
    this->pet_eating_sensor_->publish_state(false);
  if (this->abnormal_eating_sensor_ != nullptr)
    this->abnormal_eating_sensor_->publish_state(false);
  if (this->eating_speed_sensor_ != nullptr)
    this->eating_speed_sensor_->publish_state(0.0f);
  if (this->portions_today_sensor_ != nullptr)
    this->portions_today_sensor_->publish_state(this->portions_today_);
  if (this->desiccant_days_sensor_ != nullptr)
    this->desiccant_days_sensor_->publish_state(30.0f);
  if (this->cleaning_days_sensor_ != nullptr)
    this->cleaning_days_sensor_->publish_state(30.0f);
  if (this->cleaning_due_sensor_ != nullptr)
    this->cleaning_due_sensor_->publish_state(false);
  if (this->last_feed_weight_sensor_ != nullptr)
    this->last_feed_weight_sensor_->publish_state(0.0f);
  if (this->manual_feed_portions_number_ != nullptr)
    this->manual_feed_portions_number_->publish_state(1.0f);
  if (this->status_text_sensor_ != nullptr)
    this->status_text_sensor_->publish_state("Online / Standby");
  if (this->manual_feed_chime_switch_ != nullptr)
    this->manual_feed_chime_switch_->publish_state(this->storage_.manual_feed_sound);
  if (this->scheduled_feed_chime_switch_ != nullptr)
    this->scheduled_feed_chime_switch_->publish_state(this->storage_.scheduled_feed_sound);
  if (this->precision_mode_switch_ != nullptr)
    this->precision_mode_switch_->publish_state(this->storage_.precision_mode);

  // Load and restore calibration mode preference (Short 2-portion vs Stock 6-portion)
  bool saved_cal_mode = false;
  ESPPreferenceObject cal_pref = global_preferences->make_preference<bool>(fnv1_hash("ppf_cal_mode"), true);
  if (cal_pref.load(&saved_cal_mode)) {
    this->use_stock_calibration_ = saved_cal_mode;
  }
  if (this->stock_calibration_switch_ != nullptr)
    this->stock_calibration_switch_->publish_state(this->use_stock_calibration_);

  this->publish_meal_stats_();
  this->publish_tank_stats_();
}

void PPFR361SComponent::loop() {
  while (this->available()) {
    uint8_t byte;
    this->read_byte(&byte);
    this->rx_buffer_.push_back(byte);
  }

  if (!this->rx_buffer_.empty()) {
    this->parse_rx_buffer_();
  }

  uint32_t now_ms = millis();
  if (!this->is_dispensing_ && now_ms - this->last_poll_time_ >= this->poll_interval_ms_) {
    this->last_poll_time_ = now_ms;
    this->poll_status_();
  }

  if (this->is_dispensing_) {
    uint32_t elapsed = now_ms - this->dispense_start_time_;

    if (elapsed > this->dispense_timeout_ms_) {
      ESP_LOGE(TAG, "EMERGENCY SAFETY CUTOFF: Dispense duration (%lu ms) exceeded timeout (%lu ms)! Forcing motor halt...",
               (unsigned long)elapsed, (unsigned long)this->dispense_timeout_ms_);
      this->is_dispensing_ = false;

      std::vector<std::pair<uint8_t, std::vector<uint8_t>>> stop_items;
      stop_items.push_back({1, {0x00, 0x00}});
      stop_items.push_back({2, {0x00, 0x00}});
      stop_items.push_back({3, {0x00}});
      this->send_frame_(0x22, OPCODE_FEED_CMD, 0x00, stop_items);

      if (this->restore_child_lock_after_feed_) {
        this->restore_child_lock_after_feed_ = false;
        this->send_frame_(0x22, OPCODE_CHILD_LOCK, 0x00, {{1, {0x01}}});
      }

      this->emergency_stop_triggered_ = true;
      this->emergency_stop_time_ = now_ms;

      if (this->status_text_sensor_ != nullptr && this->calib_state_ == CALIB_IDLE)
        this->status_text_sensor_->publish_state("Safety Cutoff Triggered");
    }
  }

  if (this->emergency_stop_triggered_ && (now_ms - this->emergency_stop_time_ > 1500)) {
    this->emergency_stop_triggered_ = false;
    ESP_LOGW(TAG, "Dispense failsafe: rebooting coprocessor MCU via 0x5141 to cut motor PWM");
    this->send_frame_(0x22, OPCODE_REBOOT_MCU, 0x00, {});
  }

  if (this->calib_state_ == CALIB_SETTLING) {
    if (now_ms - this->calib_timer_ >= 3500) {
      this->calib_state_ = CALIB_IDLE;
      float weight = (this->bowl_weight_sensor_ != nullptr) ? this->bowl_weight_sensor_->state : 0.0f;
      if (!std::isnan(weight) && weight > 2.0f && this->calib_portions_ > 0) {
        float calculated_g = weight / static_cast<float>(this->calib_portions_);
        ESP_LOGI(TAG, "Portion calibration SUCCESS! Total %.1fg / %d portions = %.2fg/portion",
                 weight, this->calib_portions_, calculated_g);
        this->set_grams_per_portion(calculated_g);
        if (this->grams_per_portion_number_ != nullptr)
          this->grams_per_portion_number_->publish_state(calculated_g);

        char buf[64];
        snprintf(buf, sizeof(buf), "Calibrated: %.1fg/portion", calculated_g);
        if (this->status_text_sensor_ != nullptr)
          this->status_text_sensor_->publish_state(buf);
      } else {
        ESP_LOGW(TAG, "Calibration FAILED: No weight detected in bowl after dispensing (weight=%.1fg)", weight);
        if (this->status_text_sensor_ != nullptr)
          this->status_text_sensor_->publish_state("Calibration Failed: Hopper empty?");
      }
    }
  }

  if (this->calib_state_ == CALIB_STOCK_MEASURING && (now_ms - this->calib_timer_ >= 60000)) {
    ESP_LOGW(TAG, "Stock calibration timed out after 60s without coprocessor completion frame");
    this->calib_state_ = CALIB_IDLE;
    if (this->status_text_sensor_ != nullptr)
      this->status_text_sensor_->publish_state("Calibration Timed Out");
  }

  this->check_eating_settle_timeout_();

  if (this->time_ != nullptr) {
    ESPTime now = this->time_->now();
    if (now.is_valid()) {
      if (this->last_day_of_year_ == -1) {
        this->last_day_of_year_ = now.day_of_year;
      } else if (now.day_of_year != this->last_day_of_year_) {
        this->last_day_of_year_ = now.day_of_year;
        this->portions_today_ = 0;
        if (this->portions_today_sensor_ != nullptr)
          this->portions_today_sensor_->publish_state(0);
        if (this->meal_storage_.total_eaten_today_g > 0.0f) {
          ESP_LOGI(TAG, "New day (%d): Resetting daily total eaten counter to 0g", now.day_of_year);
          this->tank_storage_.past_days_consumed_g[2] = this->tank_storage_.past_days_consumed_g[1];
          this->tank_storage_.past_days_consumed_g[1] = this->tank_storage_.past_days_consumed_g[0];
          this->tank_storage_.past_days_consumed_g[0] = this->meal_storage_.total_eaten_today_g;
          this->save_tank_storage_to_nvs_();

          this->meal_storage_.total_eaten_today_g = 0.0f;
          this->meal_storage_.last_day_of_year = now.day_of_year;
          this->save_meal_storage_to_nvs_();
          if (this->total_eaten_today_sensor_ != nullptr)
            this->total_eaten_today_sensor_->publish_state(0.0f);
          this->calculate_and_publish_days_to_empty_();
        }
      }
      if (now.minute != this->last_scheduled_minute_) {
        this->last_scheduled_minute_ = now.minute;
        this->evaluate_schedules_(now);
      }
    }
  }
}

void PPFR361SComponent::dump_config() {
  ESP_LOGCONFIG(TAG, "Pawsync PPF-R361S Feeder:");
  LOG_SENSOR("  ", "Bowl Weight", this->bowl_weight_sensor_);
  LOG_SENSOR("  ", "Battery Level", this->battery_level_sensor_);
  LOG_SENSOR("  ", "Last Feed Weight", this->last_feed_weight_sensor_);
  LOG_SENSOR("  ", "Portions Dispensed Today", this->portions_today_sensor_);
  LOG_SENSOR("  ", "Desiccant Days Remaining", this->desiccant_days_sensor_);
  LOG_SENSOR("  ", "Cleaning Days Remaining", this->cleaning_days_sensor_);
  LOG_SENSOR("  ", "Last Meal Consumed", this->last_meal_consumed_sensor_);
  LOG_SENSOR("  ", "Last Meal Duration", this->last_meal_duration_sensor_);
  LOG_SENSOR("  ", "Eating Speed", this->eating_speed_sensor_);
  LOG_SENSOR("  ", "Total Eaten Today", this->total_eaten_today_sensor_);
  LOG_SENSOR("  ", "Food in Tank", this->food_in_tank_sensor_);
  LOG_SENSOR("  ", "Food Tank Level", this->food_tank_level_sensor_);
  LOG_SENSOR("  ", "Days to Empty Tank", this->days_to_empty_tank_sensor_);
  LOG_BINARY_SENSOR("  ", "Power Adapter (AC)", this->power_adapter_sensor_);
  LOG_BINARY_SENSOR("  ", "Food Hopper Low", this->food_low_sensor_);
  LOG_BINARY_SENSOR("  ", "Food Hopper Empty", this->food_empty_sensor_);
  LOG_BINARY_SENSOR("  ", "Motor Jammed", this->motor_jammed_sensor_);
  LOG_BINARY_SENSOR("  ", "Bowl Overflow", this->bowl_overflow_sensor_);
  LOG_BINARY_SENSOR("  ", "Scale Overload", this->scale_overload_sensor_);
  LOG_BINARY_SENSOR("  ", "Bowl Connected", this->bowl_connected_sensor_);
  LOG_BINARY_SENSOR("  ", "Scale Tray Connected", this->scale_tray_connected_sensor_);
  LOG_BINARY_SENSOR("  ", "Bowl Present", this->bowl_present_sensor_);
  LOG_BINARY_SENSOR("  ", "Battery Installed", this->battery_installed_sensor_);
  LOG_BINARY_SENSOR("  ", "Battery Low", this->battery_low_sensor_);
  LOG_BINARY_SENSOR("  ", "Cleaning Due", this->cleaning_due_sensor_);
  LOG_BINARY_SENSOR("  ", "Pet Eating", this->pet_eating_sensor_);
  LOG_BINARY_SENSOR("  ", "Abnormal Eating Detected", this->abnormal_eating_sensor_);
  LOG_TEXT_SENSOR("  ", "Feeder Status", this->status_text_sensor_);
  LOG_TEXT_SENSOR("  ", "Next Scheduled Meal", this->next_meal_sensor_);
  LOG_TEXT_SENSOR("  ", "Schedule Summary", this->schedule_summary_sensor_);
  LOG_TEXT_SENSOR("  ", "Last Meal Time", this->last_meal_time_sensor_);
  LOG_TEXT_SENSOR("  ", "Last Adjusted Meal", this->last_adjusted_meal_sensor_);
  LOG_TEXT_SENSOR("  ", "Last Adjusted Meal Notes", this->last_adjusted_meal_notes_sensor_);
  LOG_TEXT_SENSOR("  ", "Food Tank Category", this->food_tank_category_sensor_);
  LOG_TEXT_SENSOR("  ", "Last Refill Time", this->last_refill_time_sensor_);
  LOG_SWITCH("  ", "Child Lock", this->child_lock_switch_);
  LOG_SWITCH("  ", "Slow Feed", this->slow_feed_switch_);
  LOG_SWITCH("  ", "Sleep Mode", this->sleep_mode_switch_);
  LOG_SWITCH("  ", "Intelligent Feeding", this->intelligent_feeding_switch_);
  LOG_SWITCH("  ", "Stock Calibration Mode", this->stock_calibration_switch_);
  LOG_SWITCH("  ", "Tank Projection Mode", this->tank_projection_mode_switch_);
  LOG_SWITCH("  ", "Manual Feed Chime", this->manual_feed_chime_switch_);
  LOG_SWITCH("  ", "Scheduled Feed Chime", this->scheduled_feed_chime_switch_);
  LOG_SWITCH("  ", "Precision Mode", this->precision_mode_switch_);
  LOG_NUMBER("  ", "Manual Feed Portions", this->manual_feed_portions_number_);
  LOG_NUMBER("  ", "Tank Refill Volume", this->tank_refill_volume_number_);
  LOG_NUMBER("  ", "Tank Refill Weight", this->tank_refill_weight_number_);

  ESP_LOGCONFIG(TAG, "  On-Device Schedule: Active Plan %u, Portion Density: %.1fg/portion, Intelligent: %s",
                this->storage_.active_plan_idx + 1,
                this->storage_.grams_per_portion,
                this->storage_.intelligent_feeding ? "YES" : "NO");
}

void PPFR361SComponent::start_backup_http_server() {
  if (this->backup_server_ != nullptr) {
    this->backup_server_->start();
  }
}

void PPFR361SComponent::stop_backup_http_server() {
  if (this->backup_server_ != nullptr) {
    this->backup_server_->stop();
  }
}

}  // namespace ppf_r361s
}  // namespace esphome
