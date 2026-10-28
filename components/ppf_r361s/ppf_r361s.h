#pragma once

/**
 * ============================================================================
 * Pawsync PPF-R361S Smart Pet Feeder - Dual-MCU UART Bridge & Component
 * ============================================================================
 * 
 * Hardware Architecture:
 * - Master SoC: ESP32-C3 (RISC-V) running ESPHome.
 *   Handles Wi-Fi, mDNS, Home Assistant native API, SNTP RTC time synchronization,
 *   audio playback over I2S to NS4168 amplifier, and NVS-backed schedule storage.
 * - Coprocessor: Sinowealth SC95F8766P running factory firmware.
 *   Directly controls the motor H-bridge, strain gauge load cell (bowl weight scale),
 *   IR food hopper level sensors, battery management, and front physical buttons.
 *
 * Communication Protocol:
 * - Physical link: UART 115200 baud, 8N1 (GPIO18 RX / GPIO19 TX).
 * - Framing: Header byte 0xA5, frame type (0x21/0x22 request, 0x12 ACK), sequence ID,
 *   payload length (uint16 LE), checksum (modulo 256 sum inversion = 0xFF).
 * - Payload: Protocol Version 0x02, Opcode (uint16 LE), Status, followed by KLV items:
 *   [Key ID (1 Byte)] [Length (1 Byte)] [Value (Length Bytes)].
 *
 * Verified Reverse-Engineered Functions (Stock app_active.elf & APK):
 * - pet_feeder_uart_set_food_output (0x42009d76) -> Opcode 0x5590
 * - pet_feeder_food_output_situation_handle_klv (0x4200a2c2) -> Opcode 0x5596
 * - pet_feeder_report_zero_result_event (0x4200a5d0) -> Opcode 0x55A3
 * - pet_feeder_measuring_feed_handle_klv (0x4200a6e0) -> Opcode 0x5599
 * - savePetSingleMealMeasure (0x42009aca) -> Opcode 0x5599 (Key 1=0, Key 2=grams)
 * ============================================================================
 */

#include "esphome/core/component.h"
#include "esphome/core/automation.h"
#include "esphome/core/helpers.h"
#include "esphome/core/time.h"
#include "esphome/core/preferences.h"
#include "esphome/components/uart/uart.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/binary_sensor/binary_sensor.h"
#include "esphome/components/switch/switch.h"
#include "esphome/components/button/button.h"
#include "esphome/components/number/number.h"
#include "esphome/components/text_sensor/text_sensor.h"
#include "esphome/components/time/real_time_clock.h"

#include <vector>
#include <map>
#include <ctime>
#include <cmath>
#include <cstring>

namespace esphome {
namespace ppf_r361s {

static const uint8_t FRAME_HEADER = 0xA5;

// Protocol Opcodes (verified against stock RISC-V ELF disassembly)
static const uint16_t OPCODE_WIFI_LED           = 0x5018; // pet_feeder_uart_set_wifi_led
static const uint16_t OPCODE_PRODUCTION_STATUS  = 0x5104; // pet_feeder_uart_set_production_status
static const uint16_t OPCODE_PRODUCTION_RESULT  = 0x5105; // pet_feeder_uart_set_production_result
static const uint16_t OPCODE_TEST_CMD           = 0x5107; // pet_feeder_uart_test_cmd
static const uint16_t OPCODE_CHILD_LOCK         = 0x5140; // pet_feeder_uart_set_childlock
static const uint16_t OPCODE_REBOOT_MCU         = 0x5141; // pet_feeder_uart_reboot_mcu
static const uint16_t OPCODE_SET_DISPLAY        = 0x5504; // pet_feeder_uart_set_display
static const uint16_t OPCODE_FEED_CMD           = 0x5590; // pet_feeder_uart_set_food_output
static const uint16_t OPCODE_STATUS_POLL        = 0x5592; // pet_feeder_uart_query_device_status
static const uint16_t OPCODE_STATUS_REPORT      = 0x5593; // pet_feeder_uart_report_status
static const uint16_t OPCODE_DISPENSE_PROGRESS  = 0x5594; // pet_feeder_eating_situation_klv
static const uint16_t OPCODE_CLOCK_REQ          = 0x5595; // Coprocessor requests system timestamp
static const uint16_t OPCODE_DISPENSE_RESULT    = 0x5596; // pet_feeder_food_output_situation_handle_klv
static const uint16_t OPCODE_BUTTON_EVENT       = 0x5597; // Hardware button event
static const uint16_t OPCODE_PLAY_AUDIO         = 0x5598; // pet_feeder_play_audio_handle_klv
static const uint16_t OPCODE_MEASURING          = 0x5599; // pet_feeder_uart_set_measuring
static const uint16_t OPCODE_SLOW_FEED          = 0x55A0; // pet_feeder_uart_set_slow_mode
static const uint16_t OPCODE_SLEEP_MODE         = 0x55A0; // pet_feeder_uart_set_sleep_mode
static const uint16_t OPCODE_CLOCK_SYNC         = 0x55A1; // pet_feeder_uart_set_timestamp
static const uint16_t OPCODE_FAC_RESULT         = 0x55A2; // pet_feeder_uart_set_fac_result
static const uint16_t OPCODE_TARE_BOWL          = 0x55A3; // pet_feeder_uart_set_bowl_to_zero
static const uint16_t OPCODE_MOTOR_DIAGNOSTICS  = 0x55A4; // Coprocessor motor diagnostics
static const uint16_t OPCODE_SCALE_RAW_ADC      = 0x55A5; // Coprocessor raw ADC tracking
static const uint16_t OPCODE_PLAN_SYNC          = 0x55A6; // pet_feeder_uart_set_temperature_correct

// Schedule Storage Specifications (Matching Stock Capabilities)
static const size_t MAX_PLANS = 7;
static const size_t MAX_MEALS_PER_PLAN = 10;

enum PortionCalibrationState {
  CALIB_IDLE,
  CALIB_TARING,
  CALIB_DISPENSING,
  CALIB_SETTLING,
  CALIB_STOCK_MEASURING
};

enum EatingState : uint8_t {
  EATING_IDLE = 0,
  EATING_IN_PROGRESS = 1
};

struct MealSessionStorage {
  uint32_t magic{0x4D45414C}; // 'MEAL'
  uint32_t last_meal_epoch{0};
  uint16_t last_meal_duration_s{0};
  float last_meal_consumed_g{0.0f};
  float total_eaten_today_g{0.0f};
  uint16_t last_day_of_year{0};
  float last_eating_speed_g_per_min{0.0f};
};

struct ScheduleMeal {
  bool enabled{false};
  uint8_t hour{0};      // 0..23
  uint8_t minute{0};    // 0..59
  uint8_t portions{1};  // 1..20
  uint8_t days{0x7F};   // Bitmask: bit 0=Sun, 1=Mon, ..., 6=Sat (0x7F = Daily)
  bool sound{true};
};

struct FeedingPlan {
  char name[16]{"Plan"};
  uint8_t repeat_days{0x7F}; // Days this plan is active (bitmask)
  bool enabled{true};
  ScheduleMeal meals[MAX_MEALS_PER_PLAN];
};

struct ScheduleStorage {
  uint32_t magic{0x50534348}; // 'PSCH' (Pawsync Schedule)
  uint8_t active_plan_idx{0};  // 0..6
  bool intelligent_feeding{false};
  float grams_per_portion{8.0f};
  uint32_t last_desiccant_reset_epoch{0};
  uint32_t last_cleaning_reset_epoch{0};
  bool manual_feed_sound{true};
  bool scheduled_feed_sound{true};
  bool precision_mode{false};
  FeedingPlan plans[MAX_PLANS];
};

struct TankStorage {
  uint32_t magic{0x54414E4B};            // 'TANK'
  float initial_tank_weight_g{1600.0f};  // Base weight when refilled
  float current_food_in_tank_g{1600.0f}; // Current remaining weight
  float tank_calibration_volume_l{4.0f}; // Refill volume setting (default 4.0L)
  float tank_calibration_weight_g{0.0f}; // Optional explicit weight (0 = derive from volume)
  float reserve_constant_g{150.0f};      // Optical sensor threshold clamp (default 150g)
  uint32_t last_refill_epoch{0};
  uint32_t last_adjusted_meal_epoch{0};
  char last_adjusted_meal_notes[64]{"None"};
  char last_adjusted_meal_time_str[32]{"Never"};
  bool projection_use_rolling_avg{false};
  float past_days_consumed_g[3]{0.0f, 0.0f, 0.0f};
  uint16_t last_consumption_day{0};
};

class PPFR361SComponent : public Component, public uart::UARTDevice {
 public:
  void setup() override;
  void loop() override;
  void dump_config() override;
  float get_setup_priority() const override { return setup_priority::LATE; }

  // Entity setters - Sensors
  void set_bowl_weight_sensor(sensor::Sensor *s) { this->bowl_weight_sensor_ = s; }
  void set_battery_level_sensor(sensor::Sensor *s) { this->battery_level_sensor_ = s; }
  void set_last_feed_weight_sensor(sensor::Sensor *s) { this->last_feed_weight_sensor_ = s; }
  void set_portions_today_sensor(sensor::Sensor *s) { this->portions_today_sensor_ = s; }
  void set_desiccant_days_sensor(sensor::Sensor *s) { this->desiccant_days_sensor_ = s; }
  void set_cleaning_days_sensor(sensor::Sensor *s) { this->cleaning_days_sensor_ = s; }
  void set_last_meal_consumed_sensor(sensor::Sensor *s) { this->last_meal_consumed_sensor_ = s; }
  void set_last_meal_duration_sensor(sensor::Sensor *s) { this->last_meal_duration_sensor_ = s; }
  void set_eating_speed_sensor(sensor::Sensor *s) { this->eating_speed_sensor_ = s; }
  void set_total_eaten_today_sensor(sensor::Sensor *s) { this->total_eaten_today_sensor_ = s; }
  void set_food_in_tank_sensor(sensor::Sensor *s) { this->food_in_tank_sensor_ = s; }
  void set_food_tank_level_sensor(sensor::Sensor *s) { this->food_tank_level_sensor_ = s; }
  void set_days_to_empty_tank_sensor(sensor::Sensor *s) { this->days_to_empty_tank_sensor_ = s; }

  // Entity setters - Binary Sensors
  void set_power_adapter_sensor(binary_sensor::BinarySensor *s) { this->power_adapter_sensor_ = s; }
  void set_food_low_sensor(binary_sensor::BinarySensor *s) { this->food_low_sensor_ = s; }
  void set_food_empty_sensor(binary_sensor::BinarySensor *s) { this->food_empty_sensor_ = s; }
  void set_motor_jammed_sensor(binary_sensor::BinarySensor *s) { this->motor_jammed_sensor_ = s; }
  void set_bowl_overflow_sensor(binary_sensor::BinarySensor *s) { this->bowl_overflow_sensor_ = s; }
  void set_bowl_connected_sensor(binary_sensor::BinarySensor *s) { this->bowl_connected_sensor_ = s; }
  void set_scale_tray_connected_sensor(binary_sensor::BinarySensor *s) { this->scale_tray_connected_sensor_ = s; }
  void set_bowl_present_sensor(binary_sensor::BinarySensor *s) { this->bowl_present_sensor_ = s; }
  void set_battery_installed_sensor(binary_sensor::BinarySensor *s) { this->battery_installed_sensor_ = s; }
  void set_battery_low_sensor(binary_sensor::BinarySensor *s) { this->battery_low_sensor_ = s; }
  void set_scale_overload_sensor(binary_sensor::BinarySensor *s) { this->scale_overload_sensor_ = s; }
  void set_cleaning_due_sensor(binary_sensor::BinarySensor *s) { this->cleaning_due_sensor_ = s; }
  void set_pet_eating_sensor(binary_sensor::BinarySensor *s) { this->pet_eating_sensor_ = s; }
  void set_abnormal_eating_sensor(binary_sensor::BinarySensor *s) { this->abnormal_eating_sensor_ = s; }

  // Entity setters - Text Sensors
  void set_status_text_sensor(text_sensor::TextSensor *s) { this->status_text_sensor_ = s; }
  void set_next_meal_sensor(text_sensor::TextSensor *s) { this->next_meal_sensor_ = s; }
  void set_schedule_summary_sensor(text_sensor::TextSensor *s) { this->schedule_summary_sensor_ = s; }
  void set_last_meal_time_sensor(text_sensor::TextSensor *s) { this->last_meal_time_sensor_ = s; }
  void set_last_adjusted_meal_sensor(text_sensor::TextSensor *s) { this->last_adjusted_meal_sensor_ = s; }
  void set_last_adjusted_meal_notes_sensor(text_sensor::TextSensor *s) { this->last_adjusted_meal_notes_sensor_ = s; }
  void set_food_tank_category_sensor(text_sensor::TextSensor *s) { this->food_tank_category_sensor_ = s; }
  void set_last_refill_time_sensor(text_sensor::TextSensor *s) { this->last_refill_time_sensor_ = s; }

  // Entity setters - Switches
  void set_child_lock_switch(switch_::Switch *s) { this->child_lock_switch_ = s; }
  void set_slow_feed_switch(switch_::Switch *s) { this->slow_feed_switch_ = s; }
  void set_sleep_mode_switch(switch_::Switch *s) { this->sleep_mode_switch_ = s; }
  void set_intelligent_feeding_switch(switch_::Switch *s) { this->intelligent_feeding_switch_ = s; }
  void set_stock_calibration_switch(switch_::Switch *s) { this->stock_calibration_switch_ = s; }
  void set_tank_projection_mode_switch(switch_::Switch *s) { this->tank_projection_mode_switch_ = s; }
  void set_manual_feed_chime_switch(switch_::Switch *s) { this->manual_feed_chime_switch_ = s; }
  void set_scheduled_feed_chime_switch(switch_::Switch *s) { this->scheduled_feed_chime_switch_ = s; }
  void set_precision_mode_switch(switch_::Switch *s) { this->precision_mode_switch_ = s; }

  // Calibration Mode Configuration
  void set_use_stock_calibration(bool use_stock);
  bool get_use_stock_calibration() const { return this->use_stock_calibration_; }

  // Entity setters - Numbers
  void set_manual_feed_portions_number(number::Number *n) { this->manual_feed_portions_number_ = n; }
  void set_grams_per_portion_number(number::Number *n) { this->grams_per_portion_number_ = n; }
  void set_tank_refill_volume_number(number::Number *n) { this->tank_refill_volume_number_ = n; }
  void set_tank_refill_weight_number(number::Number *n) { this->tank_refill_weight_number_ = n; }

  // Tank Management APIs
  void reset_tank_food();
  void set_tank_refill_volume(float volume_l);
  void set_tank_refill_weight(float weight_g);
  void set_tank_projection_mode(bool use_rolling_avg);

  // Event Callback Registrations
  void add_on_meal_eaten_callback(std::function<void(float, uint32_t, float)> &&callback) {
    this->meal_eaten_callback_.add(std::move(callback));
  }
  void add_on_feed_completed_callback(std::function<void(uint16_t, float)> &&callback) {
    this->feed_completed_callback_.add(std::move(callback));
  }
  void add_on_tank_low_callback(std::function<void(float, float)> &&callback) {
    this->tank_low_callback_.add(std::move(callback));
  }

  // Time provider
  void set_time(time::RealTimeClock *time) { this->time_ = time; }

  // Public control APIs
  void feed(uint16_t portions, bool sound = true);
  float get_manual_feed_portions() const {
    return this->manual_feed_portions_number_ != nullptr ? this->manual_feed_portions_number_->state : 1.0f;
  }
  void feed_manual_portions() {
    this->feed(static_cast<uint16_t>(this->get_manual_feed_portions()), true);
  }
  void tare_bowl();
  void reset_desiccant();
  void reset_cleaning();
  void set_manual_feed_sound(bool sound);
  void set_scheduled_feed_sound(bool sound);
  void set_precision_mode(bool precision);
  bool get_manual_feed_sound() const { return this->storage_.manual_feed_sound; }
  bool get_scheduled_feed_sound() const { return this->storage_.scheduled_feed_sound; }
  bool get_precision_mode() const { return this->storage_.precision_mode; }
  void set_child_lock(bool locked);
  void set_slow_feed(bool slow);
  void set_sleep_mode(bool sleep);
  void set_wifi_led(uint8_t mode);
  void sync_clock(uint32_t epoch_sec = 0);

  // Automated Kibble Calibration Wizard
  void start_portion_calibration(uint16_t portions = 2);
  void start_stock_measuring();

  // Dual-Boot Wireless Rollback
  void rollback_to_stock_firmware();

  // Schedule Engine APIs (Up to 7 Plans, 10 Meals per Plan)
  void set_active_plan(uint8_t plan_idx);
  uint8_t get_active_plan() const { return this->storage_.active_plan_idx; }
  void set_meal_schedule(uint8_t plan_idx, uint8_t meal_idx, bool enabled, uint8_t hour, uint8_t minute,
                         uint8_t portions, uint8_t days, bool sound = true);
  void set_intelligent_feeding(bool enabled);
  bool get_intelligent_feeding() const { return this->storage_.intelligent_feeding; }
  void set_grams_per_portion(float grams);
  float get_grams_per_portion() const { return this->storage_.grams_per_portion; }
  void clear_plan(uint8_t plan_idx);
  const ScheduleMeal &get_meal(uint8_t plan_idx, uint8_t meal_idx) const;
  void sync_schedules_to_coprocessor();
  void add_on_schedule_trigger_callback(std::function<void()> callback) {
    this->schedule_trigger_callback_.add(std::move(callback));
  }

  // Human-Readable Schedule String Parsing Helpers
  uint8_t parse_days_string(const std::string &str);
  bool parse_time_string(const std::string &str, uint8_t &hour, uint8_t &minute);

 protected:
  CallbackManager<void()> schedule_trigger_callback_{};

  // Serial frame handling
  void parse_rx_buffer_();
  bool process_frame_(const uint8_t *frame, size_t len);
  void handle_opcode_(uint16_t opcode, uint8_t status, const std::map<uint8_t, std::vector<uint8_t>> &klv);

  // Serial transmission
  void send_frame_(uint8_t frame_type, uint16_t opcode,
                   uint8_t status, const std::vector<std::pair<uint8_t, std::vector<uint8_t>>> &klv_items);
  void send_ack_(uint8_t rx_seq, uint16_t opcode, uint8_t status = 0);

  uint8_t calculate_checksum_(const uint8_t *buf, size_t len);

  // Polling
  void poll_status_();

  // On-device Schedule Engine
  void load_schedule_from_nvs_();
  void save_schedule_to_nvs_();
  void evaluate_schedules_(const ESPTime &now);
  void update_schedule_sensors_();

  // On-device Meal Consumption Engine
  void load_meal_storage_from_nvs_();
  void save_meal_storage_to_nvs_();
  void process_bowl_weight_sample_(float current_weight);
  void check_eating_settle_timeout_();
  void publish_meal_stats_();

  // On-device Tank Accounting Engine
  void load_tank_storage_from_nvs_();
  void save_tank_storage_to_nvs_();
  void publish_tank_stats_();
  void calculate_and_publish_days_to_empty_();

  // Internal state
  std::vector<uint8_t> rx_buffer_;
  uint8_t tx_seq_{0};
  uint32_t last_poll_time_{0};
  uint32_t poll_interval_ms_{30000};
  bool ac_power_{true};
  uint32_t portions_today_{0};
  int last_scheduled_minute_{-1};
  bool is_dispensing_{false};
  uint32_t dispense_start_time_{0};
  uint32_t last_feed_complete_time_{0};
  uint16_t dispense_requested_portions_{1};
  uint32_t dispense_timeout_ms_{25000};
  bool restore_child_lock_after_feed_{false};
  bool emergency_stop_triggered_{false};
  uint32_t emergency_stop_time_{0};
  uint16_t last_mcu_total_portions_{0};
  bool mcu_total_initialized_{false};
  int last_day_of_year_{-1};

  // Calibration State Machine
  PortionCalibrationState calib_state_{CALIB_IDLE};
  uint32_t calib_timer_{0};
  uint16_t calib_portions_{3};
  bool use_stock_calibration_{false};

  // Schedule Storage in NVS
  ScheduleStorage storage_;

  // Meal Consumption & Eating Session State Machine
  EatingState eating_state_{EATING_IDLE};
  float baseline_bowl_weight_{0.0f};
  float eating_start_weight_{0.0f};
  float eating_last_weight_{0.0f};
  uint32_t eating_start_ms_{0};
  uint32_t eating_last_change_ms_{0};
  float drift_candidate_weight_{0.0f};
  uint32_t drift_candidate_start_ms_{0};
  float refill_candidate_weight_{0.0f};
  uint32_t refill_candidate_start_ms_{0};
  bool feed_just_finished_{false};
  MealSessionStorage meal_storage_;
  TankStorage tank_storage_;

  // Time component
  time::RealTimeClock *time_{nullptr};

  // Entity pointers
  sensor::Sensor *bowl_weight_sensor_{nullptr};
  sensor::Sensor *battery_level_sensor_{nullptr};
  sensor::Sensor *last_feed_weight_sensor_{nullptr};
  sensor::Sensor *portions_today_sensor_{nullptr};
  sensor::Sensor *desiccant_days_sensor_{nullptr};
  sensor::Sensor *cleaning_days_sensor_{nullptr};
  sensor::Sensor *last_meal_consumed_sensor_{nullptr};
  sensor::Sensor *last_meal_duration_sensor_{nullptr};
  sensor::Sensor *eating_speed_sensor_{nullptr};
  sensor::Sensor *total_eaten_today_sensor_{nullptr};
  sensor::Sensor *food_in_tank_sensor_{nullptr};
  sensor::Sensor *food_tank_level_sensor_{nullptr};
  sensor::Sensor *days_to_empty_tank_sensor_{nullptr};

  binary_sensor::BinarySensor *power_adapter_sensor_{nullptr};
  binary_sensor::BinarySensor *food_low_sensor_{nullptr};
  binary_sensor::BinarySensor *food_empty_sensor_{nullptr};
  binary_sensor::BinarySensor *motor_jammed_sensor_{nullptr};
  binary_sensor::BinarySensor *bowl_overflow_sensor_{nullptr};
  binary_sensor::BinarySensor *bowl_connected_sensor_{nullptr};
  binary_sensor::BinarySensor *scale_tray_connected_sensor_{nullptr};
  binary_sensor::BinarySensor *bowl_present_sensor_{nullptr};
  binary_sensor::BinarySensor *battery_installed_sensor_{nullptr};
  binary_sensor::BinarySensor *battery_low_sensor_{nullptr};
  binary_sensor::BinarySensor *scale_overload_sensor_{nullptr};
  binary_sensor::BinarySensor *cleaning_due_sensor_{nullptr};
  binary_sensor::BinarySensor *pet_eating_sensor_{nullptr};
  binary_sensor::BinarySensor *abnormal_eating_sensor_{nullptr};

  text_sensor::TextSensor *status_text_sensor_{nullptr};
  text_sensor::TextSensor *next_meal_sensor_{nullptr};
  text_sensor::TextSensor *schedule_summary_sensor_{nullptr};
  text_sensor::TextSensor *last_meal_time_sensor_{nullptr};
  text_sensor::TextSensor *last_adjusted_meal_sensor_{nullptr};
  text_sensor::TextSensor *last_adjusted_meal_notes_sensor_{nullptr};
  text_sensor::TextSensor *food_tank_category_sensor_{nullptr};
  text_sensor::TextSensor *last_refill_time_sensor_{nullptr};

  switch_::Switch *child_lock_switch_{nullptr};
  switch_::Switch *slow_feed_switch_{nullptr};
  switch_::Switch *sleep_mode_switch_{nullptr};
  switch_::Switch *intelligent_feeding_switch_{nullptr};
  switch_::Switch *stock_calibration_switch_{nullptr};
  switch_::Switch *tank_projection_mode_switch_{nullptr};
  switch_::Switch *manual_feed_chime_switch_{nullptr};
  switch_::Switch *scheduled_feed_chime_switch_{nullptr};
  switch_::Switch *precision_mode_switch_{nullptr};

  number::Number *manual_feed_portions_number_{nullptr};
  number::Number *grams_per_portion_number_{nullptr};
  number::Number *tank_refill_volume_number_{nullptr};
  number::Number *tank_refill_weight_number_{nullptr};

  CallbackManager<void(float, uint32_t, float)> meal_eaten_callback_{};
  CallbackManager<void(uint16_t, float)> feed_completed_callback_{};
  CallbackManager<void(float, float)> tank_low_callback_{};
};

// Child switch classes
class PPFR361SChildLockSwitch : public switch_::Switch, public Parented<PPFR361SComponent> {
 protected:
  void write_state(bool state) override {
    this->parent_->set_child_lock(state);
  }
};

class PPFR361SSlowFeedSwitch : public switch_::Switch, public Parented<PPFR361SComponent> {
 protected:
  void write_state(bool state) override {
    this->parent_->set_slow_feed(state);
  }
};

class PPFR361SSleepModeSwitch : public switch_::Switch, public Parented<PPFR361SComponent> {
 protected:
  void write_state(bool state) override {
    this->parent_->set_sleep_mode(state);
  }
};

class PPFR361SIntelligentFeedingSwitch : public switch_::Switch, public Parented<PPFR361SComponent> {
 protected:
  void write_state(bool state) override {
    this->parent_->set_intelligent_feeding(state);
  }
};

class PPFR361SStockCalibrationSwitch : public switch_::Switch, public Parented<PPFR361SComponent> {
 protected:
  void write_state(bool state) override {
    this->parent_->set_use_stock_calibration(state);
  }
};

class PPFR361STankProjectionModeSwitch : public switch_::Switch, public Parented<PPFR361SComponent> {
 protected:
  void write_state(bool state) override {
    this->parent_->set_tank_projection_mode(state);
  }
};

class PPFR361SManualFeedChimeSwitch : public switch_::Switch, public Parented<PPFR361SComponent> {
 protected:
  void write_state(bool state) override {
    this->parent_->set_manual_feed_sound(state);
  }
};

class PPFR361SScheduledFeedChimeSwitch : public switch_::Switch, public Parented<PPFR361SComponent> {
 protected:
  void write_state(bool state) override {
    this->parent_->set_scheduled_feed_sound(state);
  }
};

class PPFR361SPrecisionModeSwitch : public switch_::Switch, public Parented<PPFR361SComponent> {
 protected:
  void write_state(bool state) override {
    this->parent_->set_precision_mode(state);
  }
};

// Child button classes
class PPFR361STareButton : public button::Button, public Parented<PPFR361SComponent> {
 protected:
  void press_action() override {
    this->parent_->tare_bowl();
  }
};

class PPFR361SFeedButton : public button::Button, public Parented<PPFR361SComponent> {
 public:
  void set_portions(uint16_t portions) { this->portions_ = portions; }
  void set_sound(bool sound) { this->sound_ = sound; }
 protected:
  void press_action() override {
    this->parent_->feed(this->portions_, this->sound_);
  }
  uint16_t portions_{1};
  bool sound_{true};
};

class PPFR361SCalibrateButton : public button::Button, public Parented<PPFR361SComponent> {
 protected:
  void press_action() override {
    if (this->parent_->get_use_stock_calibration()) {
      ESP_LOGI("button", "Calibrate button pressed: Launching Stock Calibration Mode (6-portion full cycle)...");
      this->parent_->start_stock_measuring();
    } else {
      ESP_LOGI("button", "Calibrate button pressed: Launching Short Calibration Mode (2-portion quick cycle)...");
      this->parent_->start_portion_calibration(2);
    }
  }
};

class PPFR361SResetDesiccantButton : public button::Button, public Parented<PPFR361SComponent> {
 protected:
  void press_action() override {
    this->parent_->reset_desiccant();
  }
};

class PPFR361SResetCleaningButton : public button::Button, public Parented<PPFR361SComponent> {
 protected:
  void press_action() override {
    this->parent_->reset_cleaning();
  }
};

class PPFR361SResetTankFoodButton : public button::Button, public Parented<PPFR361SComponent> {
 protected:
  void press_action() override {
    this->parent_->reset_tank_food();
  }
};

// Child number classes
class PPFR361SManualPortionsNumber : public number::Number, public Parented<PPFR361SComponent> {
 protected:
  void control(float value) override {
    this->publish_state(value);
  }
};

class PPFR361STankRefillVolumeNumber : public number::Number, public Parented<PPFR361SComponent> {
 protected:
  void control(float value) override {
    this->publish_state(value);
    this->parent_->set_tank_refill_volume(value);
  }
};

class PPFR361STankRefillWeightNumber : public number::Number, public Parented<PPFR361SComponent> {
 protected:
  void control(float value) override {
    this->publish_state(value);
    this->parent_->set_tank_refill_weight(value);
  }
};

// Action classes
template<typename... Ts> class PPFR361SFeedAction : public Action<Ts...>, public Parented<PPFR361SComponent> {
 public:
  TEMPLATABLE_VALUE(uint16_t, portions)
  TEMPLATABLE_VALUE(bool, sound)

  void play(const Ts &...x) override {
    uint16_t p = this->portions_.has_value() ? this->portions_.value(x...) : 1;
    bool s = this->sound_.has_value() ? this->sound_.value(x...) : true;
    this->parent_->feed(p, s);
  }
};

template<typename... Ts> class PPFR361STareAction : public Action<Ts...>, public Parented<PPFR361SComponent> {
 public:
  void play(const Ts &...x) override {
    this->parent_->tare_bowl();
  }
};

template<typename... Ts> class PPFR361SCalibratePortionAction : public Action<Ts...>, public Parented<PPFR361SComponent> {
 public:
  TEMPLATABLE_VALUE(uint16_t, portions)

  void play(const Ts &...x) override {
    uint16_t p = this->portions_.has_value() ? this->portions_.value(x...) : 2;
    this->parent_->start_portion_calibration(p);
  }
};

template<typename... Ts> class PPFR361SSyncClockAction : public Action<Ts...>, public Parented<PPFR361SComponent> {
 public:
  TEMPLATABLE_VALUE(uint32_t, epoch)

  void play(const Ts &...x) override {
    uint32_t ep = this->epoch_.has_value() ? this->epoch_.value(x...) : 0;
    this->parent_->sync_clock(ep);
  }
};

template<typename... Ts> class PPFR361SSetScheduleMealAction : public Action<Ts...>, public Parented<PPFR361SComponent> {
 public:
  TEMPLATABLE_VALUE(uint8_t, plan)
  TEMPLATABLE_VALUE(uint8_t, meal)
  TEMPLATABLE_VALUE(bool, enabled)
  TEMPLATABLE_VALUE(uint8_t, hour)
  TEMPLATABLE_VALUE(uint8_t, minute)
  TEMPLATABLE_VALUE(uint8_t, portions)
  TEMPLATABLE_VALUE(uint8_t, days)
  TEMPLATABLE_VALUE(bool, sound)

  void play(const Ts &...x) override {
    uint8_t p = this->plan_.has_value() ? this->plan_.value(x...) : 1;
    uint8_t m = this->meal_.has_value() ? this->meal_.value(x...) : 1;
    bool en = this->enabled_.has_value() ? this->enabled_.value(x...) : true;
    uint8_t h = this->hour_.has_value() ? this->hour_.value(x...) : 8;
    uint8_t min = this->minute_.has_value() ? this->minute_.value(x...) : 0;
    uint8_t port = this->portions_.has_value() ? this->portions_.value(x...) : 1;
    uint8_t d = this->days_.has_value() ? this->days_.value(x...) : 0x7F;
    bool s = this->sound_.has_value() ? this->sound_.value(x...) : true;

    // Convert 1-based indexing from user to 0-based internal indexing
    if (p > 0) p -= 1;
    if (m > 0) m -= 1;

    this->parent_->set_meal_schedule(p, m, en, h, min, port, d, s);
  }
};

template<typename... Ts> class PPFR361SSetActivePlanAction : public Action<Ts...>, public Parented<PPFR361SComponent> {
 public:
  TEMPLATABLE_VALUE(uint8_t, plan)

  void play(const Ts &...x) override {
    uint8_t p = this->plan_.has_value() ? this->plan_.value(x...) : 1;
    if (p > 0) p -= 1;
    this->parent_->set_active_plan(p);
  }
};

template<typename... Ts> class PPFR361SSyncSchedulesAction : public Action<Ts...>, public Parented<PPFR361SComponent> {
 public:
  void play(const Ts &...x) override {
    this->parent_->sync_schedules_to_coprocessor();
  }
};

class PPFR361SScheduleTrigger : public Trigger<> {
 public:
  explicit PPFR361SScheduleTrigger(PPFR361SComponent *parent) {
    parent->add_on_schedule_trigger_callback([this]() { this->trigger(); });
  }
};

class PPFR361SMealEatenTrigger : public Trigger<float, uint32_t, float> {
 public:
  explicit PPFR361SMealEatenTrigger(PPFR361SComponent *parent) {
    parent->add_on_meal_eaten_callback([this](float g, uint32_t s, float spd) { this->trigger(g, s, spd); });
  }
};

class PPFR361SFeedCompletedTrigger : public Trigger<uint16_t, float> {
 public:
  explicit PPFR361SFeedCompletedTrigger(PPFR361SComponent *parent) {
    parent->add_on_feed_completed_callback([this](uint16_t p, float g) { this->trigger(p, g); });
  }
};

class PPFR361STankLowTrigger : public Trigger<float, float> {
 public:
  explicit PPFR361STankLowTrigger(PPFR361SComponent *parent) {
    parent->add_on_tank_low_callback([this](float rem, float days) { this->trigger(rem, days); });
  }
};

}  // namespace ppf_r361s
}  // namespace esphome
