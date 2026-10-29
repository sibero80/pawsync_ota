#include "ppf_r361s.h"
#include "esphome/core/log.h"
#include "esphome/core/hal.h"
#include <algorithm>
#include <cmath>
#include <vector>
#include <map>

#if defined(USE_ESP_IDF)
#include "esp_ota_ops.h"
#endif

namespace esphome {
namespace ppf_r361s {

static const char *const TAG = "ppf_r361s.uart";

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
          this->feed(this->calib_portions_, false);
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

        uint16_t sc = (serving_cnt > 0) ? serving_cnt : 6;
        this->set_timeout("stock_calib_refine", 3500, [this, calculated_g, sc]() {
          if (this->bowl_weight_sensor_ != nullptr && !std::isnan(this->bowl_weight_sensor_->state)) {
            float bowl_w = this->bowl_weight_sensor_->state;
            if (bowl_w > 10.0f) {
              float refined_g = bowl_w / static_cast<float>(sc);
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
  this->send_frame_(0x22, OPCODE_STATUS_POLL, 0x00, {});
}

void PPFR361SComponent::feed(uint16_t portions, bool sound) {
  if (portions == 0) portions = 1;
  if (portions > 20) portions = 20;

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
    ESP_LOGI(TAG, "Sending food output command (0x5590): target=%dg, cutoff=0g, type=%d...",
             target_weight, feed_type);
    std::vector<std::pair<uint8_t, std::vector<uint8_t>>> items;
    items.push_back({1, {static_cast<uint8_t>(target_weight & 0xFF), static_cast<uint8_t>((target_weight >> 8) & 0xFF)}});
    items.push_back({2, {0x00, 0x00}});
    items.push_back({3, {feed_type}});

    this->send_frame_(0x22, OPCODE_FEED_CMD, 0x00, items);
  };

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

  this->sync_schedules_to_coprocessor();
}

}  // namespace ppf_r361s
}  // namespace esphome
