#include "ppf_r361s.h"
#include "esphome/core/log.h"
#include "esphome/core/hal.h"
#include <algorithm>

#if defined(USE_ESP_IDF)
#include "esp_ota_ops.h"
#include "esp_partition.h"
#include "esp_system.h"
#include "esp_http_server.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#endif

namespace esphome {
namespace ppf_r361s {

static const char *const TAG = "ppf_r361s";

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
  // Read available bytes from UART buffer
  while (this->available()) {
    uint8_t byte;
    this->read_byte(&byte);
    this->rx_buffer_.push_back(byte);
  }

  if (!this->rx_buffer_.empty()) {
    this->parse_rx_buffer_();
  }

  // Periodic heartbeat / telemetry poll (skip while dispensing to avoid interrupting motor cycle)
  uint32_t now_ms = millis();
  if (!this->is_dispensing_ && now_ms - this->last_poll_time_ >= this->poll_interval_ms_) {
    this->last_poll_time_ = now_ms;
    this->poll_status_();
  }

  // Hardware Watchdog / Safety Cutoff: only triggers if coprocessor motor fails to complete
  if (this->is_dispensing_) {
    uint32_t elapsed = now_ms - this->dispense_start_time_;

    if (elapsed > this->dispense_timeout_ms_) {
      ESP_LOGE(TAG, "EMERGENCY SAFETY CUTOFF: Dispense duration (%lu ms) exceeded timeout (%lu ms)! Forcing motor halt...",
               (unsigned long)elapsed, (unsigned long)this->dispense_timeout_ms_);
      this->is_dispensing_ = false;

      // 1. Send stop command with 0x22
      std::vector<std::pair<uint8_t, std::vector<uint8_t>>> stop_items;
      stop_items.push_back({1, {0x00, 0x00}});
      stop_items.push_back({2, {0x00, 0x00}});
      stop_items.push_back({3, {0x00}});
      this->send_frame_(0x22, OPCODE_FEED_CMD, 0x00, stop_items);

      if (this->restore_child_lock_after_feed_) {
        this->restore_child_lock_after_feed_ = false;
        this->send_frame_(0x22, OPCODE_CHILD_LOCK, 0x00, {{1, {0x01}}});
      }

      // 2. Set emergency interlock to reboot coprocessor MCU (0x5141) if motor continues
      this->emergency_stop_triggered_ = true;
      this->emergency_stop_time_ = now_ms;

      if (this->status_text_sensor_ != nullptr && this->calib_state_ == CALIB_IDLE)
        this->status_text_sensor_->publish_state("Safety Cutoff Triggered");
    }
  }

  // If emergency stop was triggered and motor didn't report complete within 1.5s, hard reboot coprocessor MCU
  if (this->emergency_stop_triggered_ && (now_ms - this->emergency_stop_time_ > 1500)) {
    this->emergency_stop_triggered_ = false;
    ESP_LOGW(TAG, "Dispense failsafe: rebooting coprocessor MCU via 0x5141 to cut motor PWM");
    this->send_frame_(0x22, OPCODE_REBOOT_MCU, 0x00, {});
  }

  // Calibration Settling State Handler (measures settled kibble weight after 3.5s)
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

  // Stock Measuring timeout failsafe (60s limit)
  if (this->calib_state_ == CALIB_STOCK_MEASURING && (now_ms - this->calib_timer_ >= 60000)) {
    ESP_LOGW(TAG, "Stock calibration timed out after 60s without coprocessor completion frame");
    this->calib_state_ = CALIB_IDLE;
    if (this->status_text_sensor_ != nullptr)
      this->status_text_sensor_->publish_state("Calibration Timed Out");
  }

  // Meal eating session settle check (evaluates 60s stability window)
  this->check_eating_settle_timeout_();

  // Minute schedule tick and daily midnight portions reset
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
          // Shift 3-day consumption history for rolling average
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

uint8_t PPFR361SComponent::calculate_checksum_(const uint8_t *buf, size_t len) {
  uint32_t sum = 0;
  for (size_t i = 0; i < len; i++) {
    if (i == 5) continue; // Byte 5 is checksum placeholder
    sum += buf[i];
  }
  return static_cast<uint8_t>((~sum) & 0xFF);
}

void PPFR361SComponent::send_frame_(uint8_t frame_type, uint16_t opcode,
                                   uint8_t status, const std::vector<std::pair<uint8_t, std::vector<uint8_t>>> &klv_items) {
  // Encode inner KLV payload:
  // Byte 0: Protocol Version (Fixed 0x02)
  // Bytes 1-2: Opcode (Little-endian uint16_t)
  // Byte 3: Status code (0x00 for TX command)
  // Bytes 4+: KLV items [Key (1B)][Len (1B)][Value (Len B)]
  std::vector<uint8_t> inner;
  inner.push_back(0x02); // Fixed Protocol Version 0x02 for SC95F8766P coprocessor
  inner.push_back(static_cast<uint8_t>(opcode & 0xFF));
  inner.push_back(static_cast<uint8_t>((opcode >> 8) & 0xFF));
  inner.push_back(status);

  for (const auto &item : klv_items) {
    inner.push_back(item.first); // Key ID
    inner.push_back(static_cast<uint8_t>(item.second.size() & 0xFF)); // 1-byte length in SC95F protocol
    inner.insert(inner.end(), item.second.begin(), item.second.end());
  }

  uint16_t payload_len = static_cast<uint16_t>(inner.size());
  std::vector<uint8_t> frame;
  frame.reserve(payload_len + 6);
  frame.push_back(FRAME_HEADER);
  frame.push_back(frame_type);
  frame.push_back(this->tx_seq_++);
  frame.push_back(static_cast<uint8_t>(payload_len & 0xFF));
  frame.push_back(static_cast<uint8_t>((payload_len >> 8) & 0xFF));
  frame.push_back(0x00); // Checksum byte at index 5
  frame.insert(frame.end(), inner.begin(), inner.end());

  frame[5] = this->calculate_checksum_(frame.data(), frame.size());

  ESP_LOGI(TAG, "RAW TX FRAME (%d bytes, Opcode 0x%04X): %s", (int)frame.size(), opcode, format_hex_pretty(frame.data(), frame.size()).c_str());
  this->write_array(frame.data(), frame.size());
}

void PPFR361SComponent::send_ack_(uint8_t rx_seq, uint16_t opcode, uint8_t status) {
  std::vector<uint8_t> inner;
  inner.push_back(0x02); // Fixed Protocol Version 0x02
  inner.push_back(static_cast<uint8_t>(opcode & 0xFF));
  inner.push_back(static_cast<uint8_t>((opcode >> 8) & 0xFF));
  inner.push_back(status);

  uint16_t payload_len = static_cast<uint16_t>(inner.size());
  std::vector<uint8_t> frame;
  frame.reserve(payload_len + 6);
  frame.push_back(FRAME_HEADER);
  frame.push_back(0x12); // ACK frame type
  frame.push_back(rx_seq);
  frame.push_back(static_cast<uint8_t>(payload_len & 0xFF));
  frame.push_back(static_cast<uint8_t>((payload_len >> 8) & 0xFF));
  frame.push_back(0x00);
  frame.insert(frame.end(), inner.begin(), inner.end());
  frame[5] = this->calculate_checksum_(frame.data(), frame.size());

  ESP_LOGI(TAG, "RAW TX ACK (Seq 0x%02X, Opcode 0x%04X): %s", rx_seq, opcode, format_hex_pretty(frame.data(), frame.size()).c_str());
  this->write_array(frame.data(), frame.size());
}

void PPFR361SComponent::parse_rx_buffer_() {
  while (!this->rx_buffer_.empty()) {
    // Find header sync byte 0xA5
    if (this->rx_buffer_[0] != FRAME_HEADER) {
      this->rx_buffer_.erase(this->rx_buffer_.begin());
      continue;
    }

    if (this->rx_buffer_.size() < 6) {
      // Need at least header to read payload length
      return;
    }

    uint16_t payload_len = this->rx_buffer_[3] | (static_cast<uint16_t>(this->rx_buffer_[4]) << 8);
    if (payload_len > 256) {
      ESP_LOGW(TAG, "UART frame invalid payload length (%u > 256). Dropping sync byte.", payload_len);
      this->rx_buffer_.erase(this->rx_buffer_.begin());
      continue;
    }
    size_t full_frame_len = payload_len + 6;

    if (this->rx_buffer_.size() < full_frame_len) {
      // Waiting for complete frame to arrive
      return;
    }

    // Verify frame checksum: sum of all bytes modulo 256 must be 0xFF
    uint32_t sum = 0;
    for (size_t i = 0; i < full_frame_len; i++) {
      sum += this->rx_buffer_[i];
    }

    if ((sum & 0xFF) == 0xFF) {
      this->process_frame_(this->rx_buffer_.data(), full_frame_len);
      this->rx_buffer_.erase(this->rx_buffer_.begin(), this->rx_buffer_.begin() + full_frame_len);
    } else {
      ESP_LOGW(TAG, "Frame checksum error (modulo 256 sum: 0x%02X != 0xFF), dropping sync byte", (unsigned int)(sum & 0xFF));
      this->rx_buffer_.erase(this->rx_buffer_.begin());
    }
  }
}

bool PPFR361SComponent::process_frame_(const uint8_t *frame, size_t len) {
  if (len < 10) return false;

  ESP_LOGI(TAG, "RAW FRAME (%d bytes): %s", (int)len, format_hex_pretty(frame, len).c_str());
  uint8_t frame_type = frame[1];
  uint8_t rx_seq = frame[2];
  uint8_t proto_ver = frame[6];
  uint16_t opcode = frame[7] | (static_cast<uint16_t>(frame[8]) << 8);
  uint8_t status = frame[9];

  // Auto-ACK incoming request frames from coprocessor (frame_type 0x22)
  if (frame_type == 0x22) {
    this->send_ack_(rx_seq, opcode, 0x00);
  }

  // Parse KLV elements from frame[10] to frame[len-1]
  // In SC95F8766P hardware protocol, key length is 1 byte: [KeyID (1B)][Length (1B)][Value (Len B)]
  std::map<uint8_t, std::vector<uint8_t>> klv_map;
  size_t idx = 10;
  while (idx + 2 <= len) {
    uint8_t key_id = frame[idx];
    uint8_t item_len = frame[idx + 1];
    idx += 2;
    if (idx + item_len > len) break;
    std::vector<uint8_t> val(frame + idx, frame + idx + item_len);
    klv_map[key_id] = val;
    idx += item_len;
  }

  ESP_LOGI(TAG, "UART RX: Opcode 0x%04X, ProtoVer 0x%02X, Status %d, KLV count %d", opcode, proto_ver, status, (int)klv_map.size());
  for (const auto &kv : klv_map) {
    ESP_LOGI(TAG, "  -> KLV Key %d (len %d): %s", kv.first, (int)kv.second.size(), format_hex_pretty(kv.second).c_str());
  }
  this->handle_opcode_(opcode, status, klv_map);
  return true;
}

void PPFR361SComponent::handle_opcode_(uint16_t opcode, uint8_t status,
                                      const std::map<uint8_t, std::vector<uint8_t>> &klv) {
  switch (opcode) {
    case OPCODE_STATUS_POLL:   // 0x5592 - Device Status Query Response
    case OPCODE_STATUS_REPORT: { // 0x5593 - Full Telemetry Status Report
#if defined(USE_ESP_IDF)
      static bool ota_app_marked_valid = false;
      if (!ota_app_marked_valid) {
        esp_ota_mark_app_valid_cancel_rollback();
        ota_app_marked_valid = true;
        ESP_LOGI(TAG, "Hardware health check passed: coprocessor UART active. OTA rollback canceled & app validated.");
      }
#endif
      // Key 3: child_lock (0 = Unlocked, 1 = Locked)
      if (klv.count(3) && !klv.at(3).empty()) {
        bool locked = (klv.at(3)[0] != 0);
        if (this->child_lock_switch_ != nullptr && !this->is_dispensing_)
          this->child_lock_switch_->publish_state(locked);
      }

      // Key 4: food_remind (Food Hopper Low: 0 = OK, 1 = Low)
      if (klv.count(4) && !klv.at(4).empty()) {
        bool food_low = (klv.at(4)[0] != 0);
        if (this->food_low_sensor_ != nullptr)
          this->food_low_sensor_->publish_state(food_low);

        // Optical Sensor Self-Correction:
        // When sensor trips to LOW, clamp food remaining to reserve constant (default 150g)
        if (food_low && this->tank_storage_.current_food_in_tank_g > this->tank_storage_.reserve_constant_g) {
          ESP_LOGI(TAG, "Optical low-food sensor reached: Auto-calibrating remaining food to reserve constant (%.1fg)",
                   this->tank_storage_.reserve_constant_g);
          this->tank_storage_.current_food_in_tank_g = this->tank_storage_.reserve_constant_g;
          this->save_tank_storage_to_nvs_();
          this->publish_tank_stats_();
        }

        if (food_low) {
          float rem = this->tank_storage_.current_food_in_tank_g;
          float days = (this->days_to_empty_tank_sensor_ != nullptr) ? this->days_to_empty_tank_sensor_->state : 0.0f;
          this->tank_low_callback_.call(rem, days);
        }
      }

      // Key 5: battery_percent (0 = not installed / absent, 1-100% installed)
      if (klv.count(5) && !klv.at(5).empty()) {
        uint8_t bat = klv.at(5)[0];
        bool installed = (bat > 0 && bat <= 100);
        if (this->battery_installed_sensor_ != nullptr)
          this->battery_installed_sensor_->publish_state(installed);
        if (this->battery_level_sensor_ != nullptr) {
          if (installed) {
            this->battery_level_sensor_->publish_state(static_cast<float>(bat));
          } else {
            this->battery_level_sensor_->publish_state(NAN);
          }
        }
        if (this->battery_low_sensor_ != nullptr) {
          bool low = (!this->ac_power_ && installed && bat < 20);
          this->battery_low_sensor_->publish_state(low);
        }
      } else {
        if (this->battery_installed_sensor_ != nullptr)
          this->battery_installed_sensor_->publish_state(false);
        if (this->battery_level_sensor_ != nullptr)
          this->battery_level_sensor_->publish_state(NAN);
        if (this->battery_low_sensor_ != nullptr)
          this->battery_low_sensor_->publish_state(false);
      }

      // Update desiccant days remaining (30-day countdown matching stock APK)
      if (this->desiccant_days_sensor_ != nullptr) {
        uint32_t now_epoch = this->time_ != nullptr ? this->time_->now().timestamp : 0;
        uint8_t days_left = 30;
        if (now_epoch > 1700000000 && this->storage_.last_desiccant_reset_epoch > 0 &&
            now_epoch >= this->storage_.last_desiccant_reset_epoch) {
          uint32_t elapsed_days = (now_epoch - this->storage_.last_desiccant_reset_epoch) / 86400;
          days_left = elapsed_days >= 30 ? 0 : (30 - elapsed_days);
        }
        this->desiccant_days_sensor_->publish_state(days_left);
      }

      // Update chute cleaning days remaining (30-day countdown matching cleanFoodpassages)
      if (this->cleaning_days_sensor_ != nullptr || this->cleaning_due_sensor_ != nullptr) {
        uint32_t now_epoch = this->time_ != nullptr ? this->time_->now().timestamp : 0;
        uint8_t clean_days_left = 30;
        if (now_epoch > 1700000000 && this->storage_.last_cleaning_reset_epoch > 0 &&
            now_epoch >= this->storage_.last_cleaning_reset_epoch) {
          uint32_t elapsed_clean_days = (now_epoch - this->storage_.last_cleaning_reset_epoch) / 86400;
          clean_days_left = elapsed_clean_days >= 30 ? 0 : (30 - elapsed_clean_days);
        }
        if (this->cleaning_days_sensor_ != nullptr) {
          this->cleaning_days_sensor_->publish_state(clean_days_left);
        }
        if (this->cleaning_due_sensor_ != nullptr) {
          this->cleaning_due_sensor_->publish_state(clean_days_left == 0);
        }
      }

      // Key 6: power_mode (0 = AC Adapter, 1 = Battery Backup)
      if (klv.count(6) && !klv.at(6).empty()) {
        bool ac_power = (klv.at(6)[0] == 0);
        if (this->power_adapter_sensor_ != nullptr)
          this->power_adapter_sensor_->publish_state(ac_power);

        // Power source transition handling
        if (this->ac_power_ != ac_power) {
          this->ac_power_ = ac_power;
          if (!ac_power) {
            ESP_LOGW(TAG, "DC power unplugged: Entering Battery Power-Save Mode (throttling poll to 2min, LEDs off)");
            this->poll_interval_ms_ = 120000;
            this->set_wifi_led(0);
          } else {
            ESP_LOGI(TAG, "DC power restored: Restoring normal operational profile");
            this->poll_interval_ms_ = 30000;
            this->set_wifi_led(2);
            this->sync_clock();
          }
        }
      }

      // Key 7: error_code (0 = OK, 1 = Jammed, 2 = Chute Blocked)
      if (klv.count(7) && !klv.at(7).empty()) {
        uint8_t err = klv.at(7)[0];
        bool jammed = (err == 1);
        if (this->motor_jammed_sensor_ != nullptr)
          this->motor_jammed_sensor_->publish_state(jammed);
        if (err == 1 && this->status_text_sensor_ != nullptr)
          this->status_text_sensor_->publish_state("Motor Jammed");
        else if (err == 2 && this->status_text_sensor_ != nullptr)
          this->status_text_sensor_->publish_state("Outlet Blocked");
      }

      // Key 11 (0x0B): remaining_weight (grams, int16 LE)
      if (klv.count(11) && klv.at(11).size() >= 2) {
        int16_t weight = static_cast<int16_t>(klv.at(11)[0] | (klv.at(11)[1] << 8));
        float w_f = static_cast<float>(weight);
        if (this->bowl_weight_sensor_ != nullptr)
          this->bowl_weight_sensor_->publish_state(w_f);
        if (this->bowl_connected_sensor_ != nullptr)
          this->bowl_connected_sensor_->publish_state(true);
        if (this->scale_tray_connected_sensor_ != nullptr)
          this->scale_tray_connected_sensor_->publish_state(true);
        if (this->bowl_present_sensor_ != nullptr)
          this->bowl_present_sensor_->publish_state(true);

        // Overflow check (BLOCK_FOOD >= 150g)
        if (this->bowl_overflow_sensor_ != nullptr) {
          this->bowl_overflow_sensor_->publish_state(w_f >= 150.0f);
        }
        // Overload check (OVERLOAD > 1000g)
        if (this->scale_overload_sensor_ != nullptr) {
          this->scale_overload_sensor_->publish_state(w_f > 1000.0f);
        }

        this->process_bowl_weight_sample_(w_f);
      }

      // Only publish Online / Standby if not currently dispensing, calibrating, or holding completion status
      if (!this->is_dispensing_ && this->calib_state_ == CALIB_IDLE &&
          (millis() - this->last_feed_complete_time_ >= 5000)) {
        if (this->status_text_sensor_ != nullptr)
          this->status_text_sensor_->publish_state("Online / Standby");
      }
      break;
    }

    case OPCODE_DISPENSE_RESULT: { // 0x5596 - Dispense Complete Telemetry
      this->is_dispensing_ = false;
      this->feed_just_finished_ = true;
      this->emergency_stop_triggered_ = false;
      if (this->eating_state_ == EATING_IN_PROGRESS) {
        this->eating_state_ = EATING_IDLE;
        if (this->pet_eating_sensor_ != nullptr)
          this->pet_eating_sensor_->publish_state(false);
        this->poll_interval_ms_ = this->ac_power_ ? 30000 : 120000;
      }
      uint16_t mcu_total = 0;
      uint8_t code = 0;

      if (klv.count(2) && klv.at(2).size() >= 2)
        mcu_total = klv.at(2)[0] | (static_cast<uint16_t>(klv.at(2)[1]) << 8);
      else if (klv.count(2) && !klv.at(2).empty())
        mcu_total = klv.at(2)[0];

      if (klv.count(3) && !klv.at(3).empty())
        code = klv.at(3)[0];

      ESP_LOGI(TAG, "Feed result: weight=%dg, code=0x%02X", mcu_total, code);

      // Restore child lock if it was temporarily disengaged for remote feeding
      if (this->restore_child_lock_after_feed_) {
        this->restore_child_lock_after_feed_ = false;
        ESP_LOGI(TAG, "Restoring child lock to ON following remote dispense completion");
        this->send_frame_(0x22, OPCODE_CHILD_LOCK, 0x00, {{1, {0x01}}});
      }

      if (mcu_total == 0 && (code & 0x01) == 0) {
        ESP_LOGW(TAG, "Feed reported 0g dispensed (coprocessor locked or feed aborted).");
        this->dispense_requested_portions_ = 0;
        if (this->calib_state_ == CALIB_DISPENSING) {
          ESP_LOGE(TAG, "Calibration failed: Coprocessor locked or 0g dispensed!");
          this->calib_state_ = CALIB_IDLE;
          if (this->status_text_sensor_ != nullptr)
            this->status_text_sensor_->publish_state("Calibration Failed: Locked / 0g");
        } else if (this->status_text_sensor_ != nullptr && this->calib_state_ == CALIB_IDLE) {
          this->status_text_sensor_->publish_state("Feed Aborted (0g / Locked)");
        }
        break;
      }

      // In stock firmware (pet_feeder_food_output_situation_handle_klv):
      // Key 2 in 0x5596 is 'food_output_weight' (actual grams dispensed by coprocessor load cell)
      uint16_t grams_dispensed = mcu_total;
      float grams_per_portion = this->storage_.grams_per_portion > 0 ? this->storage_.grams_per_portion : 8.0f;
      uint16_t delta_portions = this->dispense_requested_portions_ > 0 ? this->dispense_requested_portions_ : 1;
      if (grams_dispensed > 0 && grams_per_portion > 0 && this->calib_state_ == CALIB_IDLE) {
        delta_portions = static_cast<uint16_t>(std::round(static_cast<float>(grams_dispensed) / grams_per_portion));
        if (delta_portions == 0) delta_portions = 1;
      }
      this->dispense_requested_portions_ = 0;

      // Handle Portion Calibration State Machine
      if (this->calib_state_ == CALIB_DISPENSING) {
        if (code == 0 || (code & 0x01)) {
          ESP_LOGI(TAG, "Calibration test dispense finished (%dg reported for %d test portions). Waiting 3.5s for load cell settling...",
                   grams_dispensed, this->calib_portions_);
          this->calib_state_ = CALIB_SETTLING;
          this->calib_timer_ = millis();
          if (this->status_text_sensor_ != nullptr)
            this->status_text_sensor_->publish_state("Calibrating: Settling & weighing...");
          this->set_timeout("calib_poll_1", 1500, [this]() { this->poll_status_(); });
          this->set_timeout("calib_poll_2", 2800, [this]() { this->poll_status_(); });
        } else {
          ESP_LOGE(TAG, "Calibration failed: Motor jam or chute error during test dispense!");
          this->calib_state_ = CALIB_IDLE;
          if (this->status_text_sensor_ != nullptr)
            this->status_text_sensor_->publish_state("Calibration Failed: Jam/Chute error");
        }
      }

      if (code == 0 || (code & 0x01)) {
        if (delta_portions > 0) {
          this->portions_today_ += delta_portions;
          if (this->portions_today_sensor_ != nullptr)
            this->portions_today_sensor_->publish_state(this->portions_today_);

          float grams = mcu_total > 0 ? static_cast<float>(mcu_total) : (static_cast<float>(delta_portions) * this->storage_.grams_per_portion);
          if (this->last_feed_weight_sensor_ != nullptr)
            this->last_feed_weight_sensor_->publish_state(grams);

          // Deduct from tank food accounting
          if (grams > 0.0f) {
            if (this->tank_storage_.current_food_in_tank_g >= grams) {
              this->tank_storage_.current_food_in_tank_g -= grams;
            } else {
              this->tank_storage_.current_food_in_tank_g = 0.0f;
            }
            this->save_tank_storage_to_nvs_();
            this->publish_tank_stats_();
          }

          if (this->status_text_sensor_ != nullptr && this->calib_state_ == CALIB_IDLE)
            this->status_text_sensor_->publish_state("Feeding Complete");

          this->feed_completed_callback_.call(delta_portions, grams);
        }
        if (this->motor_jammed_sensor_ != nullptr)
          this->motor_jammed_sensor_->publish_state(false);

        this->last_feed_complete_time_ = millis();
        this->set_timeout("feed_complete_reset", 5000, [this]() {
          if (!this->is_dispensing_ && this->calib_state_ == CALIB_IDLE && this->status_text_sensor_ != nullptr) {
            std::string cur = this->status_text_sensor_->state;
            if (cur.find("Calibrated:") == std::string::npos) {
              this->status_text_sensor_->publish_state("Online / Standby");
            }
          }
        });

        this->set_timeout("bowl_refresh", 2000, [this]() {
          this->poll_status_();
        });
      } else if (code == 1 || (code & 0x02)) {
        ESP_LOGE(TAG, "FEED ERROR: Motor Jammed!");
        if (this->motor_jammed_sensor_ != nullptr)
          this->motor_jammed_sensor_->publish_state(true);
        if (this->status_text_sensor_ != nullptr)
          this->status_text_sensor_->publish_state("Motor Jammed");
      } else if (code == 2 || (code & 0x04)) {
        ESP_LOGW(TAG, "FEED WARNING: Chute Blocked!");
        if (this->status_text_sensor_ != nullptr)
          this->status_text_sensor_->publish_state("Outlet Blocked");
      }
      break;
    }

    case OPCODE_FEED_CMD: { // Feed Command Response (0x5590)
      if (status == 0) {
        ESP_LOGI(TAG, "Feed request accepted by coprocessor (Status 0). Motor running...");
        this->is_dispensing_ = true;
        this->dispense_start_time_ = millis();
        if (this->status_text_sensor_ != nullptr && this->calib_state_ == CALIB_IDLE)
          this->status_text_sensor_->publish_state("Dispensing...");
      } else {
        ESP_LOGE(TAG, "Feed request rejected by coprocessor with status %d (0x%02X)", status, status);
        this->is_dispensing_ = false;
        if (this->status_text_sensor_ != nullptr)
          this->status_text_sensor_->publish_state("Feed Rejected");
      }
      break;
    }

    case OPCODE_CHILD_LOCK: { // 0x5104 - Child Lock ACK
      ESP_LOGI(TAG, "Child lock ACK from coprocessor: status %d", status);
      break;
    }

    case OPCODE_TARE_BOWL: { // 0x55A3 - Tare Result
      // In stock firmware (s_zero_result / pet_feeder_report_zero_result_event):
      // Key 1: Status Code (0 = success, 1 = bowlConnAbnormal, 2 = weightFluctuation, 3 = bowlNotPlaced)
      // Key 2: Source Flag (1 = remote app command, 0 = hardware button)
      uint8_t result = status;
      if (klv.count(1) && !klv.at(1).empty()) {
        result = klv.at(1)[0];
      }

      if (result == 0) {
        ESP_LOGI(TAG, "Bowl scale tare successful (0g confirmed)");
        this->baseline_bowl_weight_ = 0.0f;
        this->drift_candidate_weight_ = 0.0f;
        this->refill_candidate_weight_ = 0.0f;
        if (this->eating_state_ == EATING_IN_PROGRESS) {
          this->eating_state_ = EATING_IDLE;
          if (this->pet_eating_sensor_ != nullptr)
            this->pet_eating_sensor_->publish_state(false);
          this->poll_interval_ms_ = this->ac_power_ ? 30000 : 120000;
        }
        if (this->bowl_weight_sensor_ != nullptr)
          this->bowl_weight_sensor_->publish_state(0.0f);
        if (this->bowl_connected_sensor_ != nullptr)
          this->bowl_connected_sensor_->publish_state(true);
        if (this->scale_tray_connected_sensor_ != nullptr)
          this->scale_tray_connected_sensor_->publish_state(true);
        if (this->bowl_present_sensor_ != nullptr)
          this->bowl_present_sensor_->publish_state(true);

        if (this->calib_state_ == CALIB_TARING) {
          ESP_LOGI(TAG, "Calibration tare confirmed 0g. Dispensing %d test portions...", this->calib_portions_);
          if (this->status_text_sensor_ != nullptr)
            this->status_text_sensor_->publish_state("Calibrating: Dispensing test portions...");
          this->calib_state_ = CALIB_DISPENSING;
          this->calib_timer_ = millis();
          this->feed(this->calib_portions_, false); // Dispense silently
        } else {
          if (this->status_text_sensor_ != nullptr)
            this->status_text_sensor_->publish_state("Scale Tared");
        }
      } else if (result == 1) {
        ESP_LOGW(TAG, "Tare failed: Bowl connection abnormal (bowlConnAbnormal)");
        if (this->calib_state_ == CALIB_TARING) this->calib_state_ = CALIB_IDLE;
        if (this->bowl_connected_sensor_ != nullptr)
          this->bowl_connected_sensor_->publish_state(false);
        if (this->scale_tray_connected_sensor_ != nullptr)
          this->scale_tray_connected_sensor_->publish_state(false);
        if (this->status_text_sensor_ != nullptr)
          this->status_text_sensor_->publish_state("Bowl Connection Error");
      } else if (result == 2) {
        ESP_LOGW(TAG, "Tare failed: Weight Fluctuating / Unstable (weightFluctuation)");
        if (this->calib_state_ == CALIB_TARING) this->calib_state_ = CALIB_IDLE;
        if (this->status_text_sensor_ != nullptr)
          this->status_text_sensor_->publish_state("Tare Failed: Unstable Weight");
      } else if (result == 3) {
        ESP_LOGW(TAG, "Tare failed: Bowl Not Placed / Missing (bowlNotPlaced)");
        if (this->calib_state_ == CALIB_TARING) this->calib_state_ = CALIB_IDLE;
        if (this->bowl_connected_sensor_ != nullptr)
          this->bowl_connected_sensor_->publish_state(false);
        if (this->scale_tray_connected_sensor_ != nullptr)
          this->scale_tray_connected_sensor_->publish_state(true);
        if (this->bowl_present_sensor_ != nullptr)
          this->bowl_present_sensor_->publish_state(false);
        if (this->status_text_sensor_ != nullptr)
          this->status_text_sensor_->publish_state("Bowl Not Placed");
      } else {
        ESP_LOGW(TAG, "Bowl scale tare failed (result code %d)", result);
        if (this->calib_state_ == CALIB_TARING) this->calib_state_ = CALIB_IDLE;
        if (this->status_text_sensor_ != nullptr)
          this->status_text_sensor_->publish_state("Tare Failed");
      }

      this->set_timeout("tare_poll", 800, [this]() {
        this->poll_status_();
      });
      break;
    }

    case OPCODE_MEASURING: { // 0x5599 - Coprocessor Native Meal Calibration Telemetry
      if (klv.empty()) {
        ESP_LOGI(TAG, "Coprocessor ACKed measuring opcode (0x5599): status %d", status);
        break;
      }

      // In stock firmware (pet_feeder_measuring_feed_handle_klv):
      // Key 4: measuring_result (0 = success)
      // Key 2: meal_gram (uint16 LE)
      // Key 3: serving_cnt (uint16 LE)
      uint8_t measure_result = status;
      uint16_t meal_gram = 0;
      uint16_t serving_cnt = 0;

      if (klv.count(4) && !klv.at(4).empty())
        measure_result = klv.at(4)[0];
      if (klv.count(2) && klv.at(2).size() >= 2)
        meal_gram = klv.at(2)[0] | (static_cast<uint16_t>(klv.at(2)[1]) << 8);
      else if (klv.count(2) && !klv.at(2).empty())
        meal_gram = klv.at(2)[0];
      if (klv.count(3) && klv.at(3).size() >= 2)
        serving_cnt = klv.at(3)[0] | (static_cast<uint16_t>(klv.at(3)[1]) << 8);
      else if (klv.count(3) && !klv.at(3).empty())
        serving_cnt = klv.at(3)[0];

      ESP_LOGI(TAG, "Native measuring telemetry (0x5599): result=%d, gram=%dg, portions=%d",
               measure_result, meal_gram, serving_cnt);

      this->calib_state_ = CALIB_IDLE;
      this->last_feed_complete_time_ = millis();
      this->feed_just_finished_ = true;

      if (measure_result == 0 && meal_gram > 0) {
        // SC95F8766P coprocessor computes single_weight_g = total_dispensed / servings (e.g. 11g for 66g / 6).
        // Key 2 is therefore already single_weight_g.
        // If Key 2 is > 25g and serving_cnt > 1, defensively treat Key 2 as total weight across all servings.
        float calculated_g = (meal_gram > 25 && serving_cnt > 1)
                                ? (static_cast<float>(meal_gram) / static_cast<float>(serving_cnt))
                                : static_cast<float>(meal_gram);

        ESP_LOGI(TAG, "Stock single meal calibration success: %dg reported (servings=%d) -> %.2fg/portion",
                 meal_gram, serving_cnt, calculated_g);
        this->set_grams_per_portion(calculated_g);
        if (this->grams_per_portion_number_ != nullptr)
          this->grams_per_portion_number_->publish_state(calculated_g);
        char buf[64];
        snprintf(buf, sizeof(buf), "Calibrated: %.1fg/portion", calculated_g);
        if (this->status_text_sensor_ != nullptr)
          this->status_text_sensor_->publish_state(buf);

        // Schedule load-cell verification and precision refinement once bowl weight telemetry arrives
        uint16_t sc = (serving_cnt > 0) ? serving_cnt : 6;
        this->set_timeout("stock_calib_refine", 3500, [this, calculated_g, sc]() {
          if (this->bowl_weight_sensor_ != nullptr && !std::isnan(this->bowl_weight_sensor_->state)) {
            float bowl_w = this->bowl_weight_sensor_->state;
            if (bowl_w > 10.0f) {
              float refined_g = bowl_w / static_cast<float>(sc);
              // Only refine if within +/-2.5g of coprocessor reported value
              if (std::abs(refined_g - calculated_g) <= 2.5f && std::abs(refined_g - calculated_g) > 0.05f) {
                ESP_LOGI(TAG, "Refined kibble density from bowl load cell: %.1fg / %d = %.2fg/portion",
                         bowl_w, sc, refined_g);
                this->set_grams_per_portion(refined_g);
                if (this->grams_per_portion_number_ != nullptr)
                  this->grams_per_portion_number_->publish_state(refined_g);
                char rbuf[64];
                snprintf(rbuf, sizeof(rbuf), "Calibrated: %.1fg/portion", refined_g);
                if (this->status_text_sensor_ != nullptr)
                  this->status_text_sensor_->publish_state(rbuf);
              }
            }
          }
        });
      } else if (measure_result != 0) {
        ESP_LOGW(TAG, "Stock single meal calibration failed (code %d)", measure_result);
        if (this->status_text_sensor_ != nullptr)
          this->status_text_sensor_->publish_state("Calibration Failed");
      }
      break;
    }

    case OPCODE_CLOCK_REQ: { // 0x5595 - Coprocessor requests timestamp sync
      uint32_t epoch_sec = 0;
      if (this->time_ != nullptr) {
        ESPTime now = this->time_->now();
        if (now.is_valid()) epoch_sec = now.timestamp;
      }
      ESP_LOGI(TAG, "Coprocessor requested timestamp via 0x5595: syncing epoch %lu", (unsigned long)epoch_sec);
      std::vector<std::pair<uint8_t, std::vector<uint8_t>>> items;
      items.push_back({1, {
        static_cast<uint8_t>(epoch_sec & 0xFF),
        static_cast<uint8_t>((epoch_sec >> 8) & 0xFF),
        static_cast<uint8_t>((epoch_sec >> 16) & 0xFF),
        static_cast<uint8_t>((epoch_sec >> 24) & 0xFF)
      }});
      this->send_frame_(0x22, OPCODE_CLOCK_REQ, 0x00, items);
      break;
    }

    case OPCODE_BUTTON_EVENT: { // 0x5597 - Hardware Reminder / Button Notification
      uint8_t ev = 0;
      if (klv.count(7) && !klv.at(7).empty()) {
        ev = klv.at(7)[0];
      } else if (klv.count(1) && !klv.at(1).empty()) {
        ev = klv.at(1)[0];
      }
      ESP_LOGI(TAG, "Hardware event from coprocessor (0x5597): 0x%02X", ev);
      // NOTE: Coprocessor MCU directly handles front button physical dispense internally;
      // do not inject a software feed command here to avoid infinite feedback loops.
      break;
    }

    case OPCODE_PLAY_AUDIO: { // 0x5598 - Play Audio Chime Notification
      ESP_LOGI(TAG, "Coprocessor requested audio chime prompt playback (0x5598)");
      if (this->sleep_mode_switch_ != nullptr && this->sleep_mode_switch_->state) {
        ESP_LOGI(TAG, "Sleep mode active: silencing meal prompt");
      } else {
        this->schedule_trigger_callback_.call();
      }
      break;
    }

    default:
      ESP_LOGV(TAG, "Unhandled Opcode 0x%04X received", opcode);
      break;
  }
}

void PPFR361SComponent::poll_status_() {
  // Opcode 0x5592: Device Status Query (Frame type 0x22 matching stock master commands)
  this->send_frame_(0x22, OPCODE_STATUS_POLL, 0x00, {});
}

void PPFR361SComponent::feed(uint16_t portions, bool sound) {
  if (portions == 0) portions = 1;
  if (portions > 20) portions = 20;

  // Safety Check: Bowl Overflow Inhibit (BLOCK_FOOD / BLOCK_FOOD_FEEDING)
  // If the food bowl already contains >= 150g, refuse to dispense to prevent chute blockage and motor jam.
  if (this->bowl_weight_sensor_ != nullptr && this->bowl_weight_sensor_->has_state() && this->bowl_weight_sensor_->state >= 150.0f) {
    ESP_LOGW(TAG, "Bowl overflow detected (%.1fg >= 150g). Feed aborted to prevent distributor blockage.",
             this->bowl_weight_sensor_->state);
    if (this->bowl_overflow_sensor_ != nullptr)
      this->bowl_overflow_sensor_->publish_state(true);
    if (this->status_text_sensor_ != nullptr)
      this->status_text_sensor_->publish_state("Aborted: Bowl Overflow (>150g)");
    return;
  }

  float grams_per_portion = this->storage_.grams_per_portion > 0 ? this->storage_.grams_per_portion : 8.0f;
  uint16_t target_weight = static_cast<uint16_t>(std::round(portions * grams_per_portion));
  // If calibrating, ensure target_weight is sufficient to dispense all test portions without premature cutoff
  if (this->calib_state_ == CALIB_DISPENSING) {
    target_weight = static_cast<uint16_t>(std::round(portions * std::max(grams_per_portion, 11.0f)));
  }
  if (target_weight < 8) {
    target_weight = 8;
  }
  uint8_t feed_type = 0;

  ESP_LOGI(TAG, "Dispensing %d portion(s) (target %dg, type %d)...", portions, target_weight, feed_type);
  this->is_dispensing_ = true;
  this->dispense_start_time_ = millis();
  this->dispense_requested_portions_ = portions;
  this->dispense_timeout_ms_ = 25000;
  this->emergency_stop_triggered_ = false;

  if (this->status_text_sensor_ != nullptr && this->calib_state_ == CALIB_IDLE)
    this->status_text_sensor_->publish_state("Feeding...");

  auto send_feed_frame = [this, target_weight, feed_type]() {
    // Exact 3 keys matching stock protocol (pet_feeder_uart_set_food_output / executeGrain):
    // frame_type: 0x22 (SC95F protocol master request with ACK flag)
    // Key 1 (2B): Target Weight in Grams (uint16_t LE)
    // Key 2 (2B): Cutoff limit (0x0000)
    // Key 3 (1B): Feed Type (0 = remote meal command matching stock executeGrain)
    ESP_LOGI(TAG, "Sending food output command (0x5590): target=%dg, cutoff=0g, type=%d...",
             target_weight, feed_type);
    std::vector<std::pair<uint8_t, std::vector<uint8_t>>> items;
    items.push_back({1, {static_cast<uint8_t>(target_weight & 0xFF), static_cast<uint8_t>((target_weight >> 8) & 0xFF)}});
    items.push_back({2, {0x00, 0x00}});
    items.push_back({3, {feed_type}});

    this->send_frame_(0x22, OPCODE_FEED_CMD, 0x00, items);
  };

  // Interlock bypass: If child lock is active on the coprocessor, temporarily disengage it over UART
  // so remote feeding succeeds seamlessly without requiring physical interaction with the feeder button.
  if (this->child_lock_switch_ != nullptr && this->child_lock_switch_->state) {
    ESP_LOGI(TAG, "Child lock is ON: Temporarily unlocking coprocessor MCU via 0x5140 (00)...");
    this->restore_child_lock_after_feed_ = true;
    this->send_frame_(0x22, OPCODE_CHILD_LOCK, 0x00, {{1, {0x00}}});
    this->set_timeout("feed_dispatch", 60, send_feed_frame);
  } else {
    this->restore_child_lock_after_feed_ = false;
    send_feed_frame();
  }
}

void PPFR361SComponent::tare_bowl() {
  ESP_LOGI(TAG, "Zeroing bowl scale...");
  this->send_frame_(0x22, OPCODE_TARE_BOWL, 0x00, {});
  if (this->bowl_weight_sensor_ != nullptr)
    this->bowl_weight_sensor_->publish_state(0.0f);
  this->set_timeout("tare_refresh", 1500, [this]() {
    this->poll_status_();
  });
}

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

void PPFR361SComponent::set_child_lock(bool locked) {
  ESP_LOGI(TAG, "Setting child lock: %s", locked ? "ON" : "OFF");
  std::vector<std::pair<uint8_t, std::vector<uint8_t>>> items;
  items.push_back({1, {static_cast<uint8_t>(locked ? 1 : 0)}});
  this->send_frame_(0x22, OPCODE_CHILD_LOCK, 0x00, items);
  if (this->child_lock_switch_ != nullptr)
    this->child_lock_switch_->publish_state(locked);
}

void PPFR361SComponent::set_slow_feed(bool slow) {
  ESP_LOGI(TAG, "Setting slow feed mode: %s", slow ? "ON" : "OFF");
  std::vector<std::pair<uint8_t, std::vector<uint8_t>>> items;
  items.push_back({1, {static_cast<uint8_t>(slow ? 1 : 0)}});
  this->send_frame_(0x22, OPCODE_SLOW_FEED, 0x00, items);
  if (this->slow_feed_switch_ != nullptr)
    this->slow_feed_switch_->publish_state(slow);
}

void PPFR361SComponent::set_sleep_mode(bool sleep) {
  ESP_LOGI(TAG, "Setting night sleep mode: %s", sleep ? "ON" : "OFF");
  std::vector<std::pair<uint8_t, std::vector<uint8_t>>> items;
  items.push_back({1, {static_cast<uint8_t>(sleep ? 1 : 0)}});
  items.push_back({2, {0x00}});
  this->send_frame_(0x22, OPCODE_SLEEP_MODE, 0x00, items);
  if (this->sleep_mode_switch_ != nullptr)
    this->sleep_mode_switch_->publish_state(sleep);
}

void PPFR361SComponent::set_wifi_led(uint8_t mode) {
  std::vector<std::pair<uint8_t, std::vector<uint8_t>>> items;
  items.push_back({1, {mode}});
  this->send_frame_(0x22, OPCODE_WIFI_LED, 0x00, items);
}

void PPFR361SComponent::sync_clock(uint32_t epoch_sec) {
  if (epoch_sec == 0) {
    epoch_sec = static_cast<uint32_t>(::time(nullptr));
  }
  if (epoch_sec < 1700000000) {
    ESP_LOGW(TAG, "System time not yet valid (%u), skipping clock sync", (unsigned int)epoch_sec);
    return;
  }
  ESP_LOGI(TAG, "Syncing epoch clock (%u) to coprocessor...", (unsigned int)epoch_sec);
  std::vector<std::pair<uint8_t, std::vector<uint8_t>>> items;
  items.push_back({1, {
    static_cast<uint8_t>(epoch_sec & 0xFF),
    static_cast<uint8_t>((epoch_sec >> 8) & 0xFF),
    static_cast<uint8_t>((epoch_sec >> 16) & 0xFF),
    static_cast<uint8_t>((epoch_sec >> 24) & 0xFF)
  }});
  this->send_frame_(0x22, OPCODE_CLOCK_SYNC, 0x00, items);

  // Auto-sync active schedule to coprocessor following clock sync
  this->sync_schedules_to_coprocessor();
}

// On-Device Schedule Engine (Matching Stock 7 Plans x 10 Meals Capability)

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

  // Default / all days aliases
  if (s.empty() || s == "all" || s == "daily" || s == "everyday") return 0x7F;
  if (s == "weekdays" || s == "workdays") return 0x3E; // Mon-Fri (bit 1..5)
  if (s == "weekends") return 0x41; // Sat-Sun (bit 0 & 6)

  // Numeric fallback (e.g. "127")
  char *endptr = nullptr;
  long num = strtol(s.c_str(), &endptr, 10);
  if (endptr != s.c_str() && num > 0 && num <= 127) {
    return static_cast<uint8_t>(num);
  }

  uint8_t mask = 0;
  // Sunday = bit 0 (0x01)
  if (s.find("su") != std::string::npos) mask |= 0x01;
  // Monday = bit 1 (0x02)
  if (s.find("mo") != std::string::npos) mask |= 0x02;
  // Tuesday = bit 2 (0x04)
  if (s.find("tu") != std::string::npos) mask |= 0x04;
  // Wednesday = bit 3 (0x08)
  if (s.find("we") != std::string::npos) mask |= 0x08;
  // Thursday = bit 4 (0x10)
  if (s.find("th") != std::string::npos) mask |= 0x10;
  // Friday = bit 5 (0x20)
  if (s.find("fr") != std::string::npos) mask |= 0x20;
  // Saturday = bit 6 (0x40)
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
  // Schedules are evaluated locally on ESP32 in evaluate_schedules_().
  // Disassembly of app_active.elf proves Opcode 0x55A6 is pet_feeder_uart_set_temperature_correct,
  // NOT schedule synchronization. Do NOT send multi-byte schedule frames to 0x55A6.
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

    // Sync to coprocessor EEPROM (matching stock savePetSingleMealMeasure / 0x42009aca):
    // Opcode 0x5599: Key 1 = 0x00 (save command), Key 2 = single_weight_g (uint16 LE)
    uint16_t g_int = static_cast<uint16_t>(std::round(grams));
    this->send_frame_(0x22, OPCODE_MEASURING, 0x00, {
      {1, {0x00}},
      {2, {static_cast<uint8_t>(g_int & 0xFF), static_cast<uint8_t>((g_int >> 8) & 0xFF)}}
    });
  }
}

void PPFR361SComponent::set_use_stock_calibration(bool use_stock) {
  ESP_LOGI(TAG, "Setting Calibration Mode: %s", use_stock ? "Stock (6-portion full cycle)" : "Short (2-portion quick cycle)");
  this->use_stock_calibration_ = use_stock;
  if (this->stock_calibration_switch_ != nullptr)
    this->stock_calibration_switch_->publish_state(use_stock);
  ESPPreferenceObject cal_pref = global_preferences->make_preference<bool>(fnv1_hash("ppf_cal_mode"), true);
  cal_pref.save(&this->use_stock_calibration_);
}

void PPFR361SComponent::start_stock_measuring() {
  ESP_LOGI(TAG, "Sending stock coprocessor single meal measuring command (0x5599)...");
  this->calib_state_ = CALIB_STOCK_MEASURING;
  this->calib_timer_ = millis();

  if (this->status_text_sensor_ != nullptr)
    this->status_text_sensor_->publish_state("Calibrating: Measuring kibble density...");

  auto send_measure_frame = [this]() {
    // Stock parameters matching startPetSingleMealMeasure: Key 1 = 0x01 (Start measure), Key 5 = 0x00
    this->send_frame_(0x22, OPCODE_MEASURING, 0x00, {{1, {0x01}}, {5, {0x00}}});
  };

  if (this->child_lock_switch_ != nullptr && this->child_lock_switch_->state) {
    this->send_frame_(0x22, OPCODE_CHILD_LOCK, 0x00, {{1, {0x00}}});
    this->set_timeout("stock_measure_dispatch", 50, send_measure_frame);
  } else {
    send_measure_frame();
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

  // If feed just completed, lock the new post-dispense baseline
  if (this->feed_just_finished_) {
    this->feed_just_finished_ = false;
    this->baseline_bowl_weight_ = current_weight;
    this->drift_candidate_weight_ = current_weight;
    this->refill_candidate_weight_ = 0.0f;
    ESP_LOGI(TAG, "Post-dispense baseline locked at %.1fg", this->baseline_bowl_weight_);
    return;
  }

  // Baseline initialization
  if (this->baseline_bowl_weight_ <= 0.0f && current_weight > 0.0f) {
    this->baseline_bowl_weight_ = current_weight;
    this->drift_candidate_weight_ = current_weight;
    ESP_LOGD(TAG, "Bowl baseline weight established: %.1fg", this->baseline_bowl_weight_);
    return;
  }

  // Bowl removal safeguard: if weight drops to <= 0.0g (bowl lifted to clean/refill)
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
    // 1. Food added (human refill or treat dump)
    if (current_weight > this->baseline_bowl_weight_ + 2.0f) {
      // Require higher weight to persist for at least 3 seconds before locking in
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

    // 2. Detect eating start: weight drops from baseline by >= 1.5g
    float drop_from_baseline = this->baseline_bowl_weight_ - current_weight;
    if (drop_from_baseline >= 1.5f && this->baseline_bowl_weight_ >= 2.0f) {
      ESP_LOGI(TAG, "Pet eating session detected! Baseline: %.1fg, Current: %.1fg, Drop: %.1fg",
               this->baseline_bowl_weight_, current_weight, drop_from_baseline);
      this->eating_state_ = EATING_IN_PROGRESS;
      this->eating_start_weight_ = this->baseline_bowl_weight_;
      this->eating_last_weight_ = current_weight;
      this->eating_start_ms_ = now_ms;
      this->eating_last_change_ms_ = now_ms;
      this->poll_interval_ms_ = 5000; // Accelerated 5s polling during active meals

      if (this->pet_eating_sensor_ != nullptr)
        this->pet_eating_sensor_->publish_state(true);
    } else if (std::abs(current_weight - this->baseline_bowl_weight_) <= 1.0f && current_weight > 0.0f) {
      // 3. Slow ambient drift tracking: ONLY adapt after sustained stability for 3 full minutes (180,000 ms)
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
    // In progress: track continuous eating / nibbling
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
  // 60-second stabilization window (matching stock mealMeasureTimeout: 60)
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

      // Calculate eating speed in grams per minute: (consumed / duration_s) * 60.0
      float speed_g_per_min = (consumed / static_cast<float>(duration_s)) * 60.0f;
      this->meal_storage_.last_eating_speed_g_per_min = speed_g_per_min;
      if (this->eating_speed_sensor_ != nullptr) {
        this->eating_speed_sensor_->publish_state(speed_g_per_min);
      }

      // Abnormal eating anomaly check (e.g. rapid gobbling > 45g/min or lethargic < 1.5g/min)
      if (this->abnormal_eating_sensor_ != nullptr) {
        bool abnormal = (speed_g_per_min > 45.0f || speed_g_per_min < 1.5f);
        this->abnormal_eating_sensor_->publish_state(abnormal);
      }

      this->save_meal_storage_to_nvs_();
      this->publish_meal_stats_();

      // Dispatch event callback
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
  // Dynamically target the other slot regardless of whether ESPHome is currently on ota_0 or ota_1
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

#if defined(USE_ESP_IDF)
static esp_err_t http_export_status_handler(httpd_req_t *req) {
  const esp_partition_t *other = esp_ota_get_next_update_partition(nullptr);
  bool intact = false;
  const char *slot_label = "unknown";
  size_t slot_size = 0;
  char version_buf[32] = "unknown";

  if (other != nullptr) {
    slot_label = other->label;
    slot_size = other->size;
    esp_app_desc_t app_desc;
    if (esp_ota_get_partition_description(other, &app_desc) == ESP_OK) {
      if (strstr(app_desc.project_name, "pawsync-feeder") == nullptr) {
        intact = true;
        strncpy(version_buf, app_desc.version, sizeof(version_buf) - 1);
        version_buf[sizeof(version_buf) - 1] = '\0';
      }
    }
  }

  auto *comp = static_cast<PPFR361SComponent *>(req->user_ctx);
  if (comp != nullptr) {
    comp->set_stock_partition_state(intact ? STATE_STOCK_INTACT : STATE_STOCK_OVERWRITTEN);
  }
  uint8_t stock_state = (comp != nullptr) ? comp->get_stock_partition_state() : static_cast<uint8_t>(intact ? STATE_STOCK_INTACT : STATE_STOCK_OVERWRITTEN);
  bool allow_overwrite = (comp != nullptr) ? comp->is_allow_overwrite_stock() : false;

  const esp_partition_t *fac = esp_partition_find_first(ESP_PARTITION_TYPE_DATA, ESP_PARTITION_SUBTYPE_DATA_NVS, "fac_data");
  size_t fac_size = (fac != nullptr) ? fac->size : 0;

  char resp[300];
  snprintf(resp, sizeof(resp),
           "{\"stock_intact\":%s,\"stock_state\":%d,\"allow_overwrite\":%s,\"slot\":\"%s\",\"slot_size\":%u,\"fac_size\":%u,\"version\":\"%s\"}",
           intact ? "true" : "false", (int)stock_state, allow_overwrite ? "true" : "false", slot_label, (unsigned)slot_size, (unsigned)fac_size, version_buf);

  httpd_resp_set_type(req, "application/json");
  httpd_resp_send(req, resp, strlen(resp));
  return ESP_OK;
}

static esp_err_t http_export_stock_bin_handler(httpd_req_t *req) {
  const esp_partition_t *other = esp_ota_get_next_update_partition(nullptr);
  if (other == nullptr) {
    httpd_resp_send_404(req);
    return ESP_FAIL;
  }

  httpd_resp_set_type(req, "application/octet-stream");
  httpd_resp_set_hdr(req, "Content-Disposition", "attachment; filename=\"stock_partition.bin\"");

  const size_t chunk_size = 4096;
  uint8_t *chunk = (uint8_t *) malloc(chunk_size);
  if (chunk == nullptr) {
    httpd_resp_send_500(req);
    return ESP_FAIL;
  }

  size_t offset = 0;
  while (offset < other->size) {
    size_t to_read = std::min<size_t>(chunk_size, other->size - offset);
    if (esp_partition_read(other, offset, chunk, to_read) != ESP_OK) {
      free(chunk);
      if (offset == 0) {
        httpd_resp_send_500(req);
      }
      return ESP_FAIL;
    }
    if (httpd_resp_send_chunk(req, (const char *)chunk, to_read) != ESP_OK) {
      free(chunk);
      return ESP_FAIL;
    }
    offset += to_read;
    vTaskDelay(pdMS_TO_TICKS(5));
  }

  free(chunk);
  httpd_resp_send_chunk(req, nullptr, 0);
  return ESP_OK;
}

static esp_err_t http_export_calibration_handler(httpd_req_t *req) {
  const esp_partition_t *fac = esp_partition_find_first(ESP_PARTITION_TYPE_DATA, ESP_PARTITION_SUBTYPE_DATA_NVS, "fac_data");
  if (fac == nullptr) {
    httpd_resp_send_404(req);
    return ESP_FAIL;
  }

  httpd_resp_set_type(req, "application/octet-stream");
  httpd_resp_set_hdr(req, "Content-Disposition", "attachment; filename=\"fac_calibration.bin\"");

  const size_t chunk_size = 4096;
  uint8_t *chunk = (uint8_t *) malloc(chunk_size);
  if (chunk == nullptr) {
    httpd_resp_send_500(req);
    return ESP_FAIL;
  }

  size_t offset = 0;
  while (offset < fac->size) {
    size_t to_read = std::min<size_t>(chunk_size, fac->size - offset);
    if (esp_partition_read(fac, offset, chunk, to_read) != ESP_OK) {
      free(chunk);
      if (offset == 0) {
        httpd_resp_send_500(req);
      }
      return ESP_FAIL;
    }
    if (httpd_resp_send_chunk(req, (const char *)chunk, to_read) != ESP_OK) {
      free(chunk);
      return ESP_FAIL;
    }
    offset += to_read;
    vTaskDelay(pdMS_TO_TICKS(5));
  }

  free(chunk);
  httpd_resp_send_chunk(req, nullptr, 0);
  return ESP_OK;
}
#endif

void PPFR361SComponent::start_backup_http_server() {
#if defined(USE_ESP_IDF)
  if (this->backup_httpd_server_ != nullptr)
    return;

  // Initialize stock intact flag and NVS state machine by inspecting ground truth
  const esp_partition_t *other = esp_ota_get_next_update_partition(nullptr);
  esp_app_desc_t app_desc;
  uint8_t current_state = STATE_STOCK_OVERWRITTEN;
  if (other != nullptr && esp_ota_get_partition_description(other, &app_desc) == ESP_OK) {
    bool is_pawsync = (strstr(app_desc.project_name, "pawsync-feeder") != nullptr);
    current_state = is_pawsync ? STATE_STOCK_OVERWRITTEN : STATE_STOCK_INTACT;
  }
  this->set_stock_partition_state(current_state);

  httpd_config_t config = HTTPD_DEFAULT_CONFIG();
  config.server_port = 8088;
  config.ctrl_port = 32769;
  config.max_open_sockets = 1;
  config.lru_purge_enable = true;
  config.stack_size = 4096;

  httpd_handle_t server = nullptr;
  if (httpd_start(&server, &config) == ESP_OK) {
    this->backup_httpd_server_ = (void *)server;
    ESP_LOGI(TAG, "Factory backup HTTP streaming server listening on port %d", config.server_port);

    httpd_uri_t uri_status = {
      .uri = "/export_status",
      .method = HTTP_GET,
      .handler = http_export_status_handler,
      .user_ctx = this
    };
    httpd_register_uri_handler(server, &uri_status);

    httpd_uri_t uri_stock = {
      .uri = "/export_stock_bin",
      .method = HTTP_GET,
      .handler = http_export_stock_bin_handler,
      .user_ctx = this
    };
    httpd_register_uri_handler(server, &uri_stock);

    httpd_uri_t uri_cal = {
      .uri = "/export_calibration",
      .method = HTTP_GET,
      .handler = http_export_calibration_handler,
      .user_ctx = this
    };
    httpd_register_uri_handler(server, &uri_cal);
  } else {
    ESP_LOGW(TAG, "Failed to start factory backup HTTP server on port 8088");
  }
#endif
}

void PPFR361SComponent::stop_backup_http_server() {
#if defined(USE_ESP_IDF)
  if (this->backup_httpd_server_ != nullptr) {
    httpd_stop((httpd_handle_t)this->backup_httpd_server_);
    this->backup_httpd_server_ = nullptr;
    ESP_LOGI(TAG, "Factory backup HTTP streaming server stopped.");
  }
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
    // Mode B: 3-Day Rolling Average
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

  // Fallback to Mode A if rolling average is 0 or if Mode A is active
  if (daily_burn_g <= 1.0f) {
    // Mode A: Active Plan Scheduled Daily Burn
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
    // Derive from volume and portion density
    float vol_l = this->tank_storage_.tank_calibration_volume_l;
    if (vol_l <= 0.0f) vol_l = 4.0f;
    float g_per_portion = this->storage_.grams_per_portion > 0 ? this->storage_.grams_per_portion : 8.0f;
    // Standard impeller pocket volume is ~18ml per portion
    float bulk_density = g_per_portion / 18.0f; // g/ml
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
