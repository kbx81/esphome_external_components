/**
 * @file tube_clock.h
 * @brief ESPHome component for Nixie Tube Clock serial interface
 *
 * This component implements the Tube Clock Serial Remote Control API protocol,
 * providing remote access to display modes, time/date, temperature, intensity,
 * settings, hardware status, and key events via UART.
 */

#pragma once

#include "esphome/core/component.h"
#include "esphome/core/automation.h"
#include "esphome/core/hal.h"
#include "esphome/core/helpers.h"
#include "esphome/core/time.h"
#include "esphome/components/uart/uart.h"
#include <vector>
#include <string>

#ifdef USE_TUBE_CLOCK_DFU
#include "stm32flash.h"
#include "esp_http_client.h"
#endif

namespace esphome::tube_clock {

// Protocol constants
static const uint8_t PROTOCOL_HEADER_SIZE = 4;  // "$TCC" or "$TCS"
static const uint8_t MAX_MESSAGE_SIZE = 255;

// Message categories
static const char CATEGORY_PAGE = 'P';
static const char CATEGORY_KEYS = 'K';
static const char CATEGORY_HARDWARE = 'H';
static const char CATEGORY_TIME = 'T';
static const char CATEGORY_TEMPERATURE = 'M';
static const char CATEGORY_INTENSITY = 'I';
static const char CATEGORY_LED = 'L';
static const char CATEGORY_SETTINGS = 'S';
static const char CATEGORY_BUZZER = 'B';
static const char CATEGORY_TIMER = 'R';
static const char CATEGORY_DIAGNOSTICS = 'D';
static const char CATEGORY_ALARM = 'A';

// Message direction
static const char DIRECTION_COMMAND = 'C';
static const char DIRECTION_STATUS = 'S';

// Key bitmask values
enum TubeClockKey : uint8_t {
  KEY_UP = 0x01,
  KEY_DOWN = 0x02,
  KEY_ENTER = 0x04,
  KEY_C = 0x08,
  KEY_B = 0x10,
  KEY_A = 0x20,
};

// Operating modes
enum TubeClockMode : uint8_t {
  MODE_MAIN_MENU = 0,
  MODE_FIXED_DISPLAY = 1,
  MODE_TOGGLE_DISPLAY = 2,
  MODE_TIMER_COUNTER = 3,
  MODE_DMX512_DISPLAY = 4,
  MODE_SET_CLOCK = 5,
  MODE_SET_DATE = 6,
  MODE_SET_TIMER_RESET_VALUE = 7,
  MODE_SYSTEM_STATUS_VIEW = 8,
  MODE_SET_SYSTEM_OPTIONS = 9,
  MODE_SLOT_BEEP_CONFIG = 10,
  MODE_SLOT_BLINK_CONFIG = 11,
  MODE_SLOT_ONOFF_CONFIG = 12,
  MODE_SLOT_PM_INDICATOR_RGB_CONFIG = 13,
  MODE_SET_DURATION_CLOCK = 14,
  MODE_SET_DURATION_DATE = 15,
  MODE_SET_DURATION_TEMP = 16,
  MODE_SET_DURATION_FADE = 17,
  MODE_DST_BEGIN_MONTH = 18,
  MODE_DST_BEGIN_DOW_ORDINAL = 19,
  MODE_DST_END_MONTH = 20,
  MODE_DST_END_DOW_ORDINAL = 21,
  MODE_DST_SWITCH_DAY_OF_WEEK = 22,
  MODE_DST_SWITCH_HOUR = 23,
  MODE_SET_EFFECT_DURATION = 24,
  MODE_SET_EFFECT_FREQUENCY = 25,
  MODE_SET_MINIMUM_INTENSITY = 26,
  MODE_SET_BEEPER_VOLUME = 27,
  MODE_SET_TEMP_CALIBRATION = 28,
  MODE_SET_IDLE_TIMEOUT = 29,
  MODE_SET_DATE_FORMAT = 30,
  MODE_SET_TIME_ZONE = 31,
  MODE_SET_COLON_BEHAVIOR = 32,
  MODE_SET_DMX512_ADDRESS = 33,
};

// Setting indices
enum TubeClockSetting : uint8_t {
  SETTING_SYSTEM_OPTIONS = 0,
  SETTING_BEEP_STATES = 1,
  SETTING_BLINK_STATES = 2,
  SETTING_ONOFF_STATES = 3,
  SETTING_PM_INDICATOR_RED = 4,
  SETTING_PM_INDICATOR_GREEN = 5,
  SETTING_PM_INDICATOR_BLUE = 6,
  SETTING_TIME_DISPLAY_DURATION = 7,
  SETTING_DATE_DISPLAY_DURATION = 8,
  SETTING_TEMPERATURE_DISPLAY_DURATION = 9,
  SETTING_FADE_DURATION = 10,
  SETTING_DST_BEGIN_MONTH = 11,
  SETTING_DST_BEGIN_DOW_ORDINAL = 12,
  SETTING_DST_END_MONTH = 13,
  SETTING_DST_END_DOW_ORDINAL = 14,
  SETTING_DST_SWITCH_DAY_OF_WEEK = 15,
  SETTING_DST_SWITCH_HOUR = 16,
  SETTING_EFFECT_DURATION = 17,
  SETTING_EFFECT_FREQUENCY = 18,
  SETTING_MINIMUM_INTENSITY = 19,
  SETTING_BEEPER_VOLUME = 20,
  SETTING_TEMP_CALIBRATION_STM32 = 21,
  SETTING_TEMP_CALIBRATION_DS3234 = 22,
  SETTING_TEMP_CALIBRATION_DS1722 = 23,
  SETTING_TEMP_CALIBRATION_LM74 = 24,
  SETTING_IDLE_TIMEOUT = 25,
  SETTING_DATE_FORMAT = 26,
  SETTING_TIME_ZONE = 27,
  SETTING_COLON_BEHAVIOR = 28,
  SETTING_TIMER_RESET_VALUE = 29,
  SETTING_DMX_ADDRESS = 30,
};

// SystemOptions bit flags
enum TubeClockSystemOption : uint16_t {
  OPTION_DISPLAY_12_HOUR = 0x0001,
  OPTION_STATUS_LED_AS_AM_PM = 0x0002,
  OPTION_HOURLY_CHIME = 0x0004,
  OPTION_DST_ENABLE = 0x0008,
  OPTION_DISPLAY_FAHRENHEIT = 0x0010,
  OPTION_AUTO_ADJUST_INTENSITY = 0x0020,
  OPTION_STARTUP_TO_TOGGLE = 0x0040,
  OPTION_DMX_EXTENDED = 0x0080,
  OPTION_MSDS_OFF = 0x0100,
  OPTION_TRIGGER_EFFECT_ON_ROTATE = 0x0200,
};

// Hardware detection bits
enum TubeClockHardware : uint8_t {
  HARDWARE_DS3234 = 0x01,
  HARDWARE_DS1722 = 0x02,
  HARDWARE_LM74 = 0x04,
  HARDWARE_GPS_CONNECTED = 0x08,
  HARDWARE_GPS_FIX_VALID = 0x10,
};

// Temperature sources
enum TubeClockTempSource : uint8_t {
  TEMP_SOURCE_STM32_ADC = 0,
  TEMP_SOURCE_DS3234 = 1,
  TEMP_SOURCE_DS1722 = 2,
  TEMP_SOURCE_LM74 = 3,
  TEMP_SOURCE_EXTERNAL_SERIAL = 4,
};

// Fixed display modes
enum TubeClockFixedDisplayMode : uint8_t {
  FIXED_DISPLAY_TIME = 0,
  FIXED_DISPLAY_TIME_SECONDS = 1,
  FIXED_DISPLAY_DATE = 2,
  FIXED_DISPLAY_TEMPERATURE = 3,
};

// Date formats (years displayed as 2 digits only)
enum TubeClockDateFormat : uint8_t {
  DATE_FORMAT_YYMMDD = 0,
  DATE_FORMAT_DDMMYY = 1,
  DATE_FORMAT_MMDDYY = 2,
};

// Colon behaviors
enum TubeClockColonBehavior : uint8_t {
  COLON_BEHAVIOR_ON = 0,
  COLON_BEHAVIOR_OFF = 1,
  COLON_BEHAVIOR_BLINK = 2,
  COLON_BEHAVIOR_BLINK_UPPER = 3,
  COLON_BEHAVIOR_BLINK_LOWER = 4,
  COLON_BEHAVIOR_ALTERNATE_UPPER_LOWER = 5,
};

// Sentinel value for unavailable temperature
static const int16_t TEMP_UNAVAILABLE = 32767;

// Total number of settings and maximum that can be queued for startup queries
static const uint8_t SETTING_COUNT = 31;

// Number of alarm time slots
static const uint8_t ALARM_SLOT_COUNT = 8;

// Timeout for startup queries in milliseconds; retries if no response received
static const uint32_t STARTUP_TIMEOUT_MS = 2000;

// Startup state machine states
enum class StartupState : uint8_t {
  FIRMWARE,     // Waiting for firmware version query response
  PAGE,         // Waiting for page query response
  HARDWARE,     // Waiting for hardware (CON) query response
  HV_POWER,     // Waiting for HV power state query response
  TEMP_SOURCE,  // Waiting for temperature source query response
  TEMPERATURE,  // Waiting for temperature values query response
  INTENSITY,    // Waiting for intensity query response
  LED,          // Waiting for LED color query response
  GPS_STATUS,   // Waiting for GPS status query response
  SETTINGS,     // Querying pending settings one by one
  ALARM_SLOTS,  // Querying pending alarm slot times one by one
  COMPLETE,     // All startup queries complete
};

#ifdef USE_TUBE_CLOCK_DFU
// DFU (firmware update) state machine states
enum class DfuState : uint8_t {
  IDLE,                    // No firmware update in progress
  AWAITING_ACK,            // BOOT command sent, waiting for clock acknowledgement
  AWAITING_RESET_PULSE_MS, // Reset pin pulsing; waiting for pulse to complete
  FLASHING,                // Stm32Flasher running; HTTP client providing firmware chunks
};
#endif

// Forward declarations
class TubeClock;

/**
 * @brief Base class for components that listen to Tube Clock updates
 */
class TubeClockListener {
 public:
  virtual void on_tube_clock_update() = 0;
};

/**
 * @brief Main Tube Clock component class
 *
 * Handles UART communication with the Nixie Tube Clock, parsing incoming
 * messages and routing updates to registered listeners.
 */
class TubeClock : public Component, public uart::UARTDevice {
 public:
  void setup() override;
  void loop() override;
  void dump_config() override;
  float get_setup_priority() const override { return setup_priority::DATA; }

  // Register a listener for updates
  void register_listener(TubeClockListener *listener) { this->listeners_.push_back(listener); }

  // Register a setting to be queried during startup (called from sub-component setup())
  void request_setting_query(TubeClockSetting setting);

  // Register an alarm slot to be queried during startup (called from sub-component setup())
  void request_alarm_query(uint8_t slot);

  // Returns true once all startup queries have completed
  bool is_initialized() const { return this->startup_state_ == StartupState::COMPLETE; }

  // Message triggers
  void add_on_message_callback(std::function<void(const std::string &)> &&callback) {
    this->message_callback_.add(std::move(callback));
  }

  // DFU (firmware update) configuration
  void set_boot_0_pin(GPIOPin *pin) { this->boot_0_pin_ = pin; }
  void set_reset_pin(GPIOPin *pin) { this->reset_pin_ = pin; }
#ifdef USE_TUBE_CLOCK_DFU
  void set_dfu_tx_pin(int8_t pin) { this->dfu_tx_pin_ = pin; }
  void set_dfu_rx_pin(int8_t pin) { this->dfu_rx_pin_ = pin; }
  void set_dfu_baud_rate(uint32_t baud_rate) { this->dfu_baud_rate_ = baud_rate; }
  void set_dfu_parity(uart::UARTParityOptions parity) { this->dfu_parity_ = parity; }
#endif

  // Command methods
  void query_firmware_info();
  void query_gps_status();
  void query_page();
  static constexpr uint8_t NO_SUBMODE = 0xFF;
  void set_page(uint8_t mode, uint8_t submode = NO_SUBMODE);
  void query_keys();
  void query_adc();
  void query_hardware();
  void query_hv_power();
  void set_hv_power(bool on);
  void query_time();
  void set_time(uint8_t hours, uint8_t minutes, uint8_t seconds, uint16_t year, uint8_t month, uint8_t day);
  void query_temperature();
  void set_temperature(float temp_celsius);
  void query_temperature_source();
  void set_temperature_source(TubeClockTempSource source);
  void query_intensity();
  void set_intensity(uint8_t intensity);
  void set_auto_intensity(bool enable);
  void query_led();
  void set_led(uint8_t intensity, uint8_t r, uint8_t g, uint8_t b, bool gamma = true);
  void set_led_auto(bool enable);
  void query_setting(TubeClockSetting setting);
  void set_setting(TubeClockSetting setting, uint16_t value);
  void save_settings();
  void factory_reset();
  void play_rtttl(const std::string &rtttl);
  void stop_rtttl();
  void play_chime();
  void play_chime(uint8_t hour);
  void query_rtttl_status();
  void query_alarm_slot(uint8_t slot);
  void set_alarm_slot(uint8_t slot, uint8_t hour, uint8_t minute, uint8_t second);
  void timer_run_up();
  void timer_run_down();
  void timer_stop();
  void timer_reset();
  void timer_reset_to(uint32_t seconds);
  void timer_clear_alarm();
  void reset();
  void enter_bootloader();
  bool update_firmware(const std::string &url);

  // State getters
  uint8_t get_mode() const { return this->mode_; }
  uint8_t get_submode() const { return this->submode_; }
  const std::string &get_firmware_version() const { return this->firmware_version_; }
  uint32_t get_firmware_build() const { return this->firmware_build_; }
  uint8_t get_key_state() const { return this->key_state_; }
  uint16_t get_adc_light() const { return this->adc_light_; }
  uint16_t get_adc_vdda() const { return this->adc_vdda_; }
  uint16_t get_adc_vbatt() const { return this->adc_vbatt_; }
  uint8_t get_hardware_mask() const { return this->hardware_mask_; }
  bool get_hv_power() const { return this->hv_power_; }
  bool get_rtttl_playing() const { return this->rtttl_playing_; }
  bool get_timer_alarm_active() const { return this->timer_alarm_active_; }
  bool get_gps_connected() const { return this->gps_connected_; }
  bool get_gps_valid() const { return this->gps_valid_; }
  uint8_t get_gps_sats() const { return this->gps_sats_; }
  char get_timer_state() const { return this->timer_state_; }
  uint32_t get_timer_value() const { return this->timer_value_; }
  uint8_t get_intensity() const { return this->intensity_; }
  bool get_auto_intensity() const { return this->auto_intensity_; }
  uint8_t get_led_intensity() const { return this->led_intensity_; }
  uint8_t get_led_r() const { return this->led_r_; }
  uint8_t get_led_g() const { return this->led_g_; }
  uint8_t get_led_b() const { return this->led_b_; }
  bool get_led_gamma() const { return this->led_gamma_; }
  bool get_led_auto() const { return this->led_auto_; }
  int16_t get_temp_stm32() const { return this->temp_stm32_; }
  int16_t get_temp_ds3234() const { return this->temp_ds3234_; }
  int16_t get_temp_ds1722() const { return this->temp_ds1722_; }
  int16_t get_temp_lm74() const { return this->temp_lm74_; }
  int16_t get_temp_external() const { return this->temp_external_; }
  TubeClockTempSource get_temp_source() const { return this->temp_source_; }
  uint16_t get_setting(TubeClockSetting setting) const {
    if (setting < SETTING_COUNT) {
      return this->settings_[setting];
    }
    return 0;
  }
  uint8_t get_alarm_hour(uint8_t slot) const {
    return (slot >= 1 && slot <= ALARM_SLOT_COUNT) ? this->alarm_hour_[slot - 1] : 0;
  }
  uint8_t get_alarm_minute(uint8_t slot) const {
    return (slot >= 1 && slot <= ALARM_SLOT_COUNT) ? this->alarm_minute_[slot - 1] : 0;
  }
  uint8_t get_alarm_second(uint8_t slot) const {
    return (slot >= 1 && slot <= ALARM_SLOT_COUNT) ? this->alarm_second_[slot - 1] : 0;
  }

 protected:
  // Message parsing
  void process_byte_(uint8_t byte);
  void process_message_();
  bool validate_checksum_();
  uint8_t calculate_checksum_(const std::string &data);
  void parse_status_message_(char category, const std::string &data);

  // Status message handlers
  void handle_diagnostics_status_(const std::string &data);
  void handle_page_status_(const std::string &data);
  void handle_key_status_(const std::string &data);
  void handle_hardware_status_(const std::string &data);
  void handle_time_status_(const std::string &data);
  void handle_temperature_status_(const std::string &data);
  void handle_intensity_status_(const std::string &data);
  void handle_led_status_(const std::string &data);
  void handle_setting_status_(const std::string &data);
  void handle_buzzer_status_(const std::string &data);
  void handle_timer_status_(const std::string &data);
  void handle_alarm_status_(const std::string &data);
  void handle_boot_notification_(const std::string &version);

  // Message building and sending
  void send_command_(char category, const std::string &action);

  // Notify all listeners
  void notify_listeners_();

  // Startup state machine
  void send_startup_query_();
  void advance_startup_(StartupState state);
  void reset_startup_();

  // Hardware reset pin
  void pulse_reset_pin_();

  // Optional BOOT0 pin (active-high); when set, asserted around reset to enter the STM32 bootloader
  GPIOPin *boot_0_pin_{nullptr};

  // Optional hardware reset pin; pulsed after every firmware update attempt
  GPIOPin *reset_pin_{nullptr};

  // Non-blocking reset pin pulse state
  static constexpr uint32_t RESET_PULSE_MS = 50;
  static constexpr uint32_t BOOT0_RELEASE_DELAY_MS = 10;
  bool reset_asserting_{false};
  uint32_t reset_assert_time_{0};
  bool release_boot0_after_reset_{false};
  uint32_t boot0_release_time_{0};

#ifdef USE_TUBE_CLOCK_DFU
  void start_dfu_flash_();
  void tick_dfu_();
  void restore_dfu_uart_();
  uint32_t dfu_ack_deadline_ms_{0};
  std::string dfu_url_;
  DfuState dfu_state_{DfuState::IDLE};
  // DFU UART reconfiguration overrides (applied to the primary UART during DFU)
  int8_t dfu_tx_pin_{-1};                                              // -1 = no pin override
  int8_t dfu_rx_pin_{-1};                                              // -1 = no pin override
  uint32_t dfu_baud_rate_{0};                                          // 0 = no baud rate override
  uart::UARTParityOptions dfu_parity_{uart::UART_CONFIG_PARITY_EVEN};  // default for STM32 bootloader
  // DFU session state (valid only while dfu_state_ == FLASHING)
  Stm32Flasher dfu_flasher_;
  uart::UARTDevice dfu_device_;
  uint32_t dfu_original_baud_rate_{0};
  esp_http_client_handle_t dfu_http_client_{nullptr};
#endif

  // Internal state
  std::string rx_buffer_;
  std::string firmware_version_;
  uint32_t firmware_build_{0};
  bool in_message_{false};

  // Startup state machine
  StartupState startup_state_{StartupState::FIRMWARE};
  uint32_t startup_query_time_{0};
  StaticVector<TubeClockSetting, SETTING_COUNT> pending_setting_queries_;
  bool startup_waiting_{false};
  uint8_t pending_settings_index_{0};
  StaticVector<uint8_t, ALARM_SLOT_COUNT> pending_alarm_queries_;
  uint8_t pending_alarms_index_{0};

  // Callbacks
  CallbackManager<void(const std::string &)> message_callback_;

  // Listeners
  std::vector<TubeClockListener *> listeners_;

  // Cached states
  uint16_t settings_[SETTING_COUNT]{0};
  uint8_t alarm_hour_[ALARM_SLOT_COUNT]{0};
  uint8_t alarm_minute_[ALARM_SLOT_COUNT]{0};
  uint8_t alarm_second_[ALARM_SLOT_COUNT]{0};

  uint16_t adc_light_{0};
  uint16_t adc_vdda_{0};
  uint16_t adc_vbatt_{0};

  int16_t temp_stm32_{TEMP_UNAVAILABLE};
  int16_t temp_ds3234_{TEMP_UNAVAILABLE};
  int16_t temp_ds1722_{TEMP_UNAVAILABLE};
  int16_t temp_lm74_{TEMP_UNAVAILABLE};
  int16_t temp_external_{TEMP_UNAVAILABLE};

  TubeClockTempSource temp_source_{TEMP_SOURCE_STM32_ADC};

  uint8_t mode_{MODE_FIXED_DISPLAY};
  uint8_t submode_{0};
  uint8_t key_state_{0};
  uint8_t hardware_mask_{0};
  uint8_t intensity_{128};
  uint8_t led_intensity_{0};
  uint8_t led_r_{0};
  uint8_t led_g_{0};
  uint8_t led_b_{0};
  bool led_gamma_{true};
  bool led_auto_{false};
  bool hv_power_{false};
  bool rtttl_playing_{false};
  bool gps_connected_{false};
  bool gps_valid_{false};
  uint8_t gps_sats_{0};
  bool auto_intensity_{false};
  bool timer_alarm_active_{false};
  char timer_state_{'S'};
  uint32_t timer_value_{0};
};

// Automation triggers
class MessageTrigger : public Trigger<std::string> {
 public:
  explicit MessageTrigger(TubeClock *parent) {
    parent->add_on_message_callback([this](const std::string &message) { this->trigger(message); });
  }
};

// Automation actions
template<typename... Ts> class SetTemperatureAction : public Action<Ts...> {
 public:
  SetTemperatureAction(TubeClock *parent) : parent_(parent) {}

  TEMPLATABLE_VALUE(float, temperature)

  void play(const Ts &...x) override { this->parent_->set_temperature(this->temperature_.value(x...)); }

 protected:
  TubeClock *parent_;
};

template<typename... Ts> class SetTimeAction : public Action<Ts...> {
 public:
  SetTimeAction(TubeClock *parent) : parent_(parent) {}

  TEMPLATABLE_VALUE(ESPTime, time)
  TEMPLATABLE_VALUE(int, hours)
  TEMPLATABLE_VALUE(int, minutes)
  TEMPLATABLE_VALUE(int, seconds)
  TEMPLATABLE_VALUE(int, year)
  TEMPLATABLE_VALUE(int, month)
  TEMPLATABLE_VALUE(int, day)

  void set_has_time(bool has_time) { this->has_time_ = has_time; }

  void play(const Ts &...x) override {
    if (this->has_time_) {
      auto t = this->time_.value(x...);
      this->parent_->set_time(t.hour, t.minute, t.second, t.year, t.month, t.day_of_month);
    } else {
      this->parent_->set_time(this->hours_.value(x...), this->minutes_.value(x...), this->seconds_.value(x...),
                              this->year_.value(x...), this->month_.value(x...), this->day_.value(x...));
    }
  }

 protected:
  bool has_time_{false};
  TubeClock *parent_;
};

template<typename... Ts> class PlayRtttlAction : public Action<Ts...> {
 public:
  PlayRtttlAction(TubeClock *parent) : parent_(parent) {}

  TEMPLATABLE_VALUE(std::string, rtttl)

  void play(const Ts &...x) override { this->parent_->play_rtttl(this->rtttl_.value(x...)); }

 protected:
  TubeClock *parent_;
};

template<typename... Ts> class StopRtttlAction : public Action<Ts...> {
 public:
  StopRtttlAction(TubeClock *parent) : parent_(parent) {}

  void play(const Ts &...x) override { this->parent_->stop_rtttl(); }

 protected:
  TubeClock *parent_;
};

template<typename... Ts> class SaveSettingsAction : public Action<Ts...> {
 public:
  SaveSettingsAction(TubeClock *parent) : parent_(parent) {}

  void play(const Ts &...x) override { this->parent_->save_settings(); }

 protected:
  TubeClock *parent_;
};

template<typename... Ts> class FactoryResetAction : public Action<Ts...> {
 public:
  FactoryResetAction(TubeClock *parent) : parent_(parent) {}

  void play(const Ts &...x) override { this->parent_->factory_reset(); }

 protected:
  TubeClock *parent_;
};

template<typename... Ts> class ResetAction : public Action<Ts...> {
 public:
  ResetAction(TubeClock *parent) : parent_(parent) {}

  void play(const Ts &...x) override { this->parent_->reset(); }

 protected:
  TubeClock *parent_;
};

template<typename... Ts> class EnterBootloaderAction : public Action<Ts...> {
 public:
  EnterBootloaderAction(TubeClock *parent) : parent_(parent) {}

  void play(const Ts &...x) override { this->parent_->enter_bootloader(); }

 protected:
  TubeClock *parent_;
};

template<typename... Ts> class UpdateFirmwareAction : public Action<Ts...> {
 public:
  UpdateFirmwareAction(TubeClock *parent) : parent_(parent) {}

  TEMPLATABLE_VALUE(std::string, url)

  void play(const Ts &...x) override { this->parent_->update_firmware(this->url_.value(x...)); }

 protected:
  TubeClock *parent_;
};

template<typename... Ts> class TimerRunUpAction : public Action<Ts...> {
 public:
  TimerRunUpAction(TubeClock *parent) : parent_(parent) {}

  void play(const Ts &...x) override { this->parent_->timer_run_up(); }

 protected:
  TubeClock *parent_;
};

template<typename... Ts> class TimerRunDownAction : public Action<Ts...> {
 public:
  TimerRunDownAction(TubeClock *parent) : parent_(parent) {}

  void play(const Ts &...x) override { this->parent_->timer_run_down(); }

 protected:
  TubeClock *parent_;
};

template<typename... Ts> class TimerStopAction : public Action<Ts...> {
 public:
  TimerStopAction(TubeClock *parent) : parent_(parent) {}

  void play(const Ts &...x) override { this->parent_->timer_stop(); }

 protected:
  TubeClock *parent_;
};

template<typename... Ts> class TimerResetAction : public Action<Ts...> {
 public:
  TimerResetAction(TubeClock *parent) : parent_(parent) {}

  void play(const Ts &...x) override { this->parent_->timer_reset(); }

 protected:
  TubeClock *parent_;
};

template<typename... Ts> class TimerResetToAction : public Action<Ts...> {
 public:
  TimerResetToAction(TubeClock *parent) : parent_(parent) {}

  TEMPLATABLE_VALUE(uint32_t, seconds)

  void play(const Ts &...x) override { this->parent_->timer_reset_to(this->seconds_.value(x...)); }

 protected:
  TubeClock *parent_;
};

template<typename... Ts> class TimerClearAlarmAction : public Action<Ts...> {
 public:
  TimerClearAlarmAction(TubeClock *parent) : parent_(parent) {}

  void play(const Ts &...x) override { this->parent_->timer_clear_alarm(); }

 protected:
  TubeClock *parent_;
};

template<typename... Ts> class PlayChimeAction : public Action<Ts...> {
 public:
  PlayChimeAction(TubeClock *parent) : parent_(parent) {}

  TEMPLATABLE_VALUE(int, hour)

  void set_has_hour(bool has) { this->has_hour_ = has; }

  void play(const Ts &...x) override {
    if (this->has_hour_) {
      this->parent_->play_chime(static_cast<uint8_t>(this->hour_.value(x...)));
    } else {
      this->parent_->play_chime();
    }
  }

 protected:
  TubeClock *parent_;
  bool has_hour_{false};
};

}  // namespace esphome::tube_clock
