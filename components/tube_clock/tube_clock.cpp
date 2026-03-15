/**
 * @file tube_clock.cpp
 * @brief Implementation of Tube Clock component
 */

#include "tube_clock.h"
#include "esphome/core/application.h"
#include "esphome/core/log.h"
#include "esphome/core/helpers.h"
#include <cstdio>

#ifdef USE_TUBE_CLOCK_DFU
#include <esp_http_client.h>
#ifdef USE_ESP_IDF
#include "esphome/components/uart/uart_component_esp_idf.h"
#endif
#endif

namespace esphome::tube_clock {

static const char *const TAG = "tube_clock";

void TubeClock::setup() {
  if (this->boot_0_pin_ != nullptr) {
    this->boot_0_pin_->setup();
    this->boot_0_pin_->digital_write(false);  // Start low (active-high)
  }
  if (this->reset_pin_ != nullptr) {
    this->reset_pin_->setup();
    this->reset_pin_->digital_write(true);
  }
  // Startup queries are driven by the state machine in loop()
}

void TubeClock::loop() {
  // Handle non-blocking reset pin pulse
  if (this->reset_asserting_ && this->reset_pin_ != nullptr) {
    if (millis() - this->reset_assert_time_ >= RESET_PULSE_MS) {
      this->reset_pin_->digital_write(true);
      this->reset_asserting_ = false;
      this->boot0_release_time_ = millis();
    }
  }

  // Release BOOT0 no sooner than BOOT0_RELEASE_DELAY_MS after reset was released
  if (this->release_boot0_after_reset_ && !this->reset_asserting_ && this->boot_0_pin_ != nullptr) {
    if (millis() - this->boot0_release_time_ >= BOOT0_RELEASE_DELAY_MS) {
      this->boot_0_pin_->digital_write(false);
      this->release_boot0_after_reset_ = false;
    }
  }

#ifdef USE_TUBE_CLOCK_DFU
  // Reset pulse complete; BOOT0 held long enough and bootloader has had time to start.
  if (this->dfu_state_ == DfuState::AWAITING_RESET_PULSE_MS && !this->reset_asserting_ &&
      millis() - this->boot0_release_time_ >= BOOT0_RELEASE_DELAY_MS) {
    this->start_dfu_flash_();
  }
  // Non-blocking flash: drive the state machine one step per loop() iteration.
  if (this->dfu_state_ == DfuState::FLASHING) {
    this->tick_dfu_();
    return;
  }
#endif

  // Process incoming bytes from UART
  while (this->available()) {
    uint8_t byte;
    if (this->read_byte(&byte)) {
      this->process_byte_(byte);
    }
  }

  // Run startup state machine: send next query only after receiving a response for the previous one
  if (this->startup_state_ != StartupState::COMPLETE) {
    if (!this->startup_waiting_) {
      this->send_startup_query_();
    } else if ((millis() - this->startup_query_time_) > STARTUP_TIMEOUT_MS) {
      ESP_LOGW(TAG, "Query timed out (state %u)", static_cast<uint8_t>(this->startup_state_));
      this->startup_waiting_ = false;
    }
  }

#ifdef USE_TUBE_CLOCK_DFU
  // If the clock did not acknowledge the BOOT1 command within the deadline, proceed anyway
  if (this->dfu_state_ == DfuState::AWAITING_ACK && millis() > this->dfu_ack_deadline_ms_) {
    ESP_LOGW(TAG, "Timed out waiting for bootloader ACK, pulsing reset anyway");
    this->pulse_reset_pin_();
    this->dfu_state_ = DfuState::AWAITING_RESET_PULSE_MS;
  }
#endif
}

void TubeClock::dump_config() {
  ESP_LOGCONFIG(TAG, "Tube Clock:");
  this->check_uart_settings(115200, 1, uart::UART_CONFIG_PARITY_NONE, 8);
  LOG_PIN("  Boot0 Pin: ", this->boot_0_pin_);
  LOG_PIN("  Reset Pin: ", this->reset_pin_);
}

void TubeClock::process_byte_(uint8_t byte) {
  // Start of message detection
  if (byte == '$' && this->rx_buffer_.empty()) {
    this->in_message_ = true;
    this->rx_buffer_.clear();
  }

  if (this->in_message_) {
    this->rx_buffer_.push_back(byte);

    // Check for newline (end of message)
    if (byte == '\n') {
      this->process_message_();
      this->rx_buffer_.clear();
      this->in_message_ = false;
    }

    // Prevent buffer overflow
    if (this->rx_buffer_.size() > MAX_MESSAGE_SIZE) {
      ESP_LOGW(TAG, "Message buffer overflow, resetting");
      this->rx_buffer_.clear();
      this->in_message_ = false;
    }
  }
}

void TubeClock::process_message_() {
  if (this->rx_buffer_.size() < 6) {  // Minimum: "$TCS<c>\n" (header + direction + category + newline)
    ESP_LOGW(TAG, "Message too short: %zu bytes", this->rx_buffer_.size());
    return;
  }

  // Validate header
  if (this->rx_buffer_[0] != '$' || this->rx_buffer_[1] != 'T' || this->rx_buffer_[2] != 'C') {
    ESP_LOGW(TAG, "Invalid message header");
    return;
  }

  ESP_LOGV(TAG, "Received: %s", this->rx_buffer_.c_str());

  // Extract message (without leading '$' and trailing '\n')
  std::string message = this->rx_buffer_.substr(1, this->rx_buffer_.size() - 2);

  // Find checksum delimiter
  size_t checksum_pos = message.find('*');
  if (checksum_pos == std::string::npos) {
    // Message without checksum - accepted per protocol spec
    ESP_LOGV(TAG, "Message without checksum: %s", message.c_str());
  } else {
    // Validate checksum
    if (!this->validate_checksum_()) {
      ESP_LOGW(TAG, "Invalid checksum");
      return;
    }
    // Remove checksum from message
    message = message.substr(0, checksum_pos);
  }

  // Trigger message callback
  this->message_callback_.call(this->rx_buffer_);

  // Parse status messages
  // Message format: "TC<direction><category><data>"
  // Example: "TCSHADC29,3297,2782" where TC=header, S=status, H=hardware, ADC29,3297,2782=data
  if (message.size() >= 4 && message[2] == DIRECTION_STATUS) {
    char category = message[3];
    std::string data = message.substr(4);
    this->parse_status_message_(category, data);
  }
}

bool TubeClock::validate_checksum_() {
  size_t checksum_pos = this->rx_buffer_.find('*');
  if (checksum_pos == std::string::npos || checksum_pos + 3 > this->rx_buffer_.size()) {
    return false;
  }

  // Extract data between '$' and '*' (exclusive)
  std::string data = this->rx_buffer_.substr(1, checksum_pos - 1);

  // Calculate expected checksum
  uint8_t calculated = this->calculate_checksum_(data);

  // Extract provided checksum (two hex digits after '*')
  std::string checksum_str = this->rx_buffer_.substr(checksum_pos + 1, 2);
  uint8_t provided = 0;
  if (sscanf(checksum_str.c_str(), "%02hhX", &provided) != 1) {
    return false;
  }

  return calculated == provided;
}

uint8_t TubeClock::calculate_checksum_(const std::string &data) {
  uint8_t checksum = 0;
  for (char c : data) {
    checksum ^= static_cast<uint8_t>(c);
  }
  return checksum;
}

void TubeClock::parse_status_message_(char category, const std::string &data) {
  switch (category) {
    case CATEGORY_DIAGNOSTICS:
      this->handle_diagnostics_status_(data);
      break;
    case CATEGORY_PAGE:
      this->handle_page_status_(data);
      break;
    case CATEGORY_KEYS:
      this->handle_key_status_(data);
      break;
    case CATEGORY_HARDWARE:
      this->handle_hardware_status_(data);
      break;
    case CATEGORY_TIME:
      this->handle_time_status_(data);
      break;
    case CATEGORY_TEMPERATURE:
      this->handle_temperature_status_(data);
      break;
    case CATEGORY_INTENSITY:
      this->handle_intensity_status_(data);
      break;
    case CATEGORY_LED:
      this->handle_led_status_(data);
      break;
    case CATEGORY_SETTINGS:
      this->handle_setting_status_(data);
      break;
    case CATEGORY_BUZZER:
      this->handle_buzzer_status_(data);
      break;
    case CATEGORY_TIMER:
      this->handle_timer_status_(data);
      break;
    case CATEGORY_ALARM:
      this->handle_alarm_status_(data);
      break;
    default:
      ESP_LOGV(TAG, "Unknown category: %c", category);
      break;
  }
}

void TubeClock::handle_diagnostics_status_(const std::string &data) {
  // Firmware version response: "F<version>,<build>"
  if (!data.empty() && data[0] == 'F') {
    size_t comma = data.find(',', 1);
    if (comma != std::string::npos) {
      this->firmware_version_ = data.substr(1, comma - 1);
      uint32_t build = 0;
      sscanf(data.c_str() + comma + 1, "%lu", &build);
      this->firmware_build_ = build;
    } else {
      this->firmware_version_ = data.substr(1);
      this->firmware_build_ = 0;
    }
    ESP_LOGV(TAG, "Firmware: %s, build %lu", this->firmware_version_.c_str(), (unsigned long) this->firmware_build_);
    if (this->is_initialized()) {
      this->notify_listeners_();
    }
    this->advance_startup_(StartupState::FIRMWARE);
    return;
  }

  // GPS status response/notification: "GPS<connected>,<valid>,<sats>"
  if (data.size() > 3 && data[0] == 'G' && data[1] == 'P' && data[2] == 'S') {
    uint8_t connected = 0, valid = 0, sats = 0;
    if (sscanf(data.c_str() + 3, "%hhu,%hhu,%hhu", &connected, &valid, &sats) != 3) {
      return;
    }
    bool new_connected = (connected != 0);
    bool new_valid = (valid != 0);
    if (new_connected != this->gps_connected_ || new_valid != this->gps_valid_ || sats != this->gps_sats_) {
      this->gps_connected_ = new_connected;
      this->gps_valid_ = new_valid;
      this->gps_sats_ = sats;
      ESP_LOGV(TAG, "GPS: connected=%s, valid=%s, sats=%u", ONOFF(this->gps_connected_), ONOFF(this->gps_valid_),
               this->gps_sats_);
      if (this->is_initialized()) {
        this->notify_listeners_();
      }
    }
    this->advance_startup_(StartupState::GPS_STATUS);
  }
}

void TubeClock::handle_page_status_(const std::string &data) {
  // Format: "P<mode>" or "P<mode>P<submode>"
  uint8_t new_mode = 0;
  uint8_t new_submode = 0;

  size_t submode_pos = data.find('P', 1);  // Look for second 'P'
  if (submode_pos != std::string::npos) {
    // Has submode
    sscanf(data.c_str(), "%hhu P%hhu", &new_mode, &new_submode);
  } else {
    // No submode
    sscanf(data.c_str(), "%hhu", &new_mode);
  }

  bool mode_changed = (new_mode != this->mode_) || (new_submode != this->submode_);
  this->mode_ = new_mode;
  this->submode_ = new_submode;

  if (mode_changed) {
    ESP_LOGV(TAG, "Mode changed: %u, submode: %u", this->mode_, this->submode_);
    if (this->is_initialized()) {
      this->notify_listeners_();
    }
  }
  this->advance_startup_(StartupState::PAGE);
}

void TubeClock::handle_key_status_(const std::string &data) {
  // Format: "<bitmask>" — full set of currently-pressed keys, sent both as a query response
  // and as an unsolicited notification whenever the pressed-key set changes.
  uint8_t key_mask = 0;
  sscanf(data.c_str(), "%hhu", &key_mask);
  if (key_mask != this->key_state_) {
    this->key_state_ = key_mask;
    if (this->is_initialized()) {
      this->notify_listeners_();
    }
  }
}

void TubeClock::handle_hardware_status_(const std::string &data) {
  // Could be ADC data or hardware connection mask
  if (data.size() > 3 && data.substr(0, 3) == "ADC") {
    // Format: "ADC<light>,<vdda>,<vbatt>"
    uint16_t new_light = 0;
    uint16_t new_vdda = 0;
    uint16_t new_vbatt = 0;
    sscanf(data.c_str() + 3, "%hu,%hu,%hu", &new_light, &new_vdda, &new_vbatt);
    if (new_light != this->adc_light_ || new_vdda != this->adc_vdda_ || new_vbatt != this->adc_vbatt_) {
      this->adc_light_ = new_light;
      this->adc_vdda_ = new_vdda;
      this->adc_vbatt_ = new_vbatt;
      ESP_LOGV(TAG, "ADC: light=%u, vdda=%u, vbatt=%u", this->adc_light_, this->adc_vdda_, this->adc_vbatt_);
      if (this->is_initialized()) {
        this->notify_listeners_();
      }
    }
  } else if (data.size() >= 1 && data[0] == 'V') {
    // Format: "V<0|1>" - HV power status
    uint8_t power = 0;
    sscanf(data.c_str() + 1, "%hhu", &power);
    bool new_hv_power = (power != 0);
    if (new_hv_power != this->hv_power_) {
      this->hv_power_ = new_hv_power;
      ESP_LOGV(TAG, "HV power: %s", ONOFF(this->hv_power_));
      if (this->is_initialized()) {
        this->notify_listeners_();
      }
    }
    this->advance_startup_(StartupState::HV_POWER);
  } else if (data == "BOOT0") {
    ESP_LOGV(TAG, "Bootloader flag cleared");
  } else if (data == "BOOT1") {
    ESP_LOGV(TAG, "Clock set bootloader flag");
#ifdef USE_TUBE_CLOCK_DFU
    if (this->dfu_state_ == DfuState::AWAITING_ACK) {
      ESP_LOGV(TAG, "Bootloader flag acknowledged, pulsing reset");
      this->pulse_reset_pin_();
      this->dfu_state_ = DfuState::AWAITING_RESET_PULSE_MS;
    }
#endif
  } else if (data == "BOOT2") {
    ESP_LOGV(TAG, "Clock entering bootloader immediately");
  } else if (data.size() > 3 && data.substr(0, 3) == "CON") {
    // Format: "CON<bitmask>"
    uint8_t new_mask = 0;
    sscanf(data.c_str() + 3, "%hhu", &new_mask);
    if (new_mask != this->hardware_mask_) {
      this->hardware_mask_ = new_mask;
      ESP_LOGV(TAG, "Hardware mask: 0x%02X", this->hardware_mask_);
      if (this->is_initialized()) {
        this->notify_listeners_();
      }
    }
    this->advance_startup_(StartupState::HARDWARE);
  }
}

void TubeClock::handle_time_status_(const std::string &data) {
  // Format: "<HHMMSSYYYYMMDD>"
  if (data.size() >= 14) {
    // Time/date received - could be exposed via time sensor platform in future
    ESP_LOGV(TAG, "Time: %s", data.c_str());
  }
}

void TubeClock::handle_temperature_status_(const std::string &data) {
  // Could be temperature values or source setting
  if (data.size() > 0 && data[0] == 'S') {
    // Temperature source response or error ($TCSMSE = sensor not available)
    if (data.size() > 1 && data[1] == 'E') {
      ESP_LOGW(TAG, "Temperature source change rejected: sensor not available");
      return;
    }
    if (data.size() > 1) {
      uint8_t source = 0;
      sscanf(data.c_str() + 1, "%hhu", &source);
      TubeClockTempSource new_source = static_cast<TubeClockTempSource>(source);
      if (new_source != this->temp_source_) {
        this->temp_source_ = new_source;
        ESP_LOGV(TAG, "Temperature source: %u", source);
        if (this->is_initialized()) {
          this->notify_listeners_();
        }
      }
      this->advance_startup_(StartupState::TEMP_SOURCE);
    }
  } else {
    // Temperature values: "<adc>,<ds3234>,<ds1722>,<lm74>,<external>"
    int temps[5];
    if (sscanf(data.c_str(), "%d,%d,%d,%d,%d", &temps[0], &temps[1], &temps[2], &temps[3], &temps[4]) == 5) {
      int16_t new_temps[5] = {static_cast<int16_t>(temps[0]), static_cast<int16_t>(temps[1]),
                              static_cast<int16_t>(temps[2]), static_cast<int16_t>(temps[3]),
                              static_cast<int16_t>(temps[4])};
      if (new_temps[0] != this->temp_stm32_ || new_temps[1] != this->temp_ds3234_ ||
          new_temps[2] != this->temp_ds1722_ || new_temps[3] != this->temp_lm74_ ||
          new_temps[4] != this->temp_external_) {
        this->temp_stm32_ = new_temps[0];
        this->temp_ds3234_ = new_temps[1];
        this->temp_ds1722_ = new_temps[2];
        this->temp_lm74_ = new_temps[3];
        this->temp_external_ = new_temps[4];
        ESP_LOGV(TAG, "Temperatures: STM32=%d, DS3234=%d, DS1722=%d, LM74=%d, External=%d", this->temp_stm32_,
                 this->temp_ds3234_, this->temp_ds1722_, this->temp_lm74_, this->temp_external_);
        if (this->is_initialized()) {
          this->notify_listeners_();
        }
      }
      this->advance_startup_(StartupState::TEMPERATURE);
    }
  }
}

void TubeClock::handle_intensity_status_(const std::string &data) {
  // Format: "<intensity>,<auto>"
  uint16_t intensity = 0;
  uint8_t auto_mode = 0;
  if (sscanf(data.c_str(), "%hu,%hhu", &intensity, &auto_mode) != 2) {
    return;
  }
  uint8_t new_intensity = static_cast<uint8_t>(intensity);
  bool new_auto_intensity = (auto_mode != 0);

  if (new_intensity != this->intensity_ || new_auto_intensity != this->auto_intensity_) {
    this->intensity_ = new_intensity;
    this->auto_intensity_ = new_auto_intensity;
    ESP_LOGV(TAG, "Intensity: %u, auto: %s", this->intensity_, ONOFF(this->auto_intensity_));
    if (this->is_initialized()) {
      this->notify_listeners_();
    }
  }
  this->advance_startup_(StartupState::INTENSITY);
}

void TubeClock::handle_led_status_(const std::string &data) {
  // Format: "<i>,<r>,<g>,<b>,<gamma>,<auto>"
  // gamma and auto are optional (backwards-compatible with 4- or 5-field responses)
  uint16_t i = 0, r = 0, g = 0, b = 0;
  uint8_t gamma_mode = 1, auto_mode = 0;
  int parsed = sscanf(data.c_str(), "%hu,%hu,%hu,%hu,%hhu,%hhu", &i, &r, &g, &b, &gamma_mode, &auto_mode);
  if (parsed < 4) {
    return;
  }
  uint8_t new_intensity = static_cast<uint8_t>(i);
  uint8_t new_r = static_cast<uint8_t>(r);
  uint8_t new_g = static_cast<uint8_t>(g);
  uint8_t new_b = static_cast<uint8_t>(b);
  bool new_led_gamma = (gamma_mode != 0);
  bool new_led_auto = (auto_mode != 0);
  if (new_intensity != this->led_intensity_ || new_r != this->led_r_ || new_g != this->led_g_ ||
      new_b != this->led_b_ || new_led_gamma != this->led_gamma_ || new_led_auto != this->led_auto_) {
    this->led_intensity_ = new_intensity;
    this->led_r_ = new_r;
    this->led_g_ = new_g;
    this->led_b_ = new_b;
    this->led_gamma_ = new_led_gamma;
    this->led_auto_ = new_led_auto;
    ESP_LOGV(TAG, "LED: i=%u, r=%u, g=%u, b=%u, gamma=%s, auto=%s", this->led_intensity_, this->led_r_,
             this->led_g_, this->led_b_, ONOFF(this->led_gamma_), ONOFF(this->led_auto_));
    if (this->is_initialized()) {
      this->notify_listeners_();
    }
  }
  this->advance_startup_(StartupState::LED);
}

void TubeClock::handle_setting_status_(const std::string &data) {
  // Erase settings (factory reset) response: "ERASE<result>"
  if (data.size() >= 5 && data.substr(0, 5) == "ERASE") {
    uint8_t result = 0;
    if (data.size() > 5) {
      sscanf(data.c_str() + 5, "%hhu", &result);
    }
    if (result) {
      ESP_LOGI(TAG, "Settings erased (factory reset)");
    } else {
      ESP_LOGW(TAG, "Failed to erase settings");
    }
    return;
  }

  // Save settings response: "W<result>"
  if (!data.empty() && data[0] == 'W') {
    uint8_t result = 0;
    if (data.size() > 1) {
      sscanf(data.c_str() + 1, "%hhu", &result);
    }
    if (result) {
      ESP_LOGV(TAG, "Settings saved to flash");
    } else {
      ESP_LOGW(TAG, "Failed to save settings to flash");
    }
    return;
  }

  // Format: "<nn>,<value>"
  uint8_t index = 0;
  uint16_t value = 0;
  if (sscanf(data.c_str(), "%hhu,%hu", &index, &value) != 2) {
    return;
  }
  if (index < SETTING_COUNT) {
    if (this->settings_[index] != value) {
      this->settings_[index] = value;
      ESP_LOGV(TAG, "Setting %u = %u", index, value);
      if (this->is_initialized()) {
        this->notify_listeners_();
      }
    }
  }
  if (this->startup_state_ == StartupState::SETTINGS && this->startup_waiting_) {
    this->startup_waiting_ = false;
    this->pending_settings_index_++;
    if (this->pending_settings_index_ >= this->pending_setting_queries_.size()) {
      this->startup_state_ = StartupState::ALARM_SLOTS;
      ESP_LOGV(TAG, "Settings queries complete, querying alarm slots...");
    }
  }
}

void TubeClock::send_command_(char category, const std::string &action) {
  // Build message: "$TCC<category><action>*XX\n"
  std::string data = std::string("TC") + DIRECTION_COMMAND + category + action;
  uint8_t checksum = this->calculate_checksum_(data);

  char message[MAX_MESSAGE_SIZE];
  snprintf(message, sizeof(message), "$%s*%02X\n", data.c_str(), checksum);

  ESP_LOGV(TAG, "Sending: %s", message);
  this->write_str(message);
  this->flush();
}

void TubeClock::notify_listeners_() {
  for (auto *listener : this->listeners_) {
    listener->on_tube_clock_update();
  }
}

void TubeClock::send_startup_query_() {
  switch (this->startup_state_) {
    case StartupState::FIRMWARE:
      ESP_LOGV(TAG, "Querying firmware info...");
      this->send_command_(CATEGORY_DIAGNOSTICS, "F");
      break;
    case StartupState::PAGE:
      ESP_LOGV(TAG, "Querying page...");
      this->send_command_(CATEGORY_PAGE, "");
      break;
    case StartupState::HARDWARE:
      ESP_LOGV(TAG, "Querying hardware...");
      this->send_command_(CATEGORY_HARDWARE, "CON");
      break;
    case StartupState::HV_POWER:
      ESP_LOGV(TAG, "Querying HV power...");
      this->send_command_(CATEGORY_HARDWARE, "V");
      break;
    case StartupState::TEMP_SOURCE:
      ESP_LOGV(TAG, "Querying temperature source...");
      this->send_command_(CATEGORY_TEMPERATURE, "S");
      break;
    case StartupState::TEMPERATURE:
      ESP_LOGV(TAG, "Querying temperatures...");
      this->send_command_(CATEGORY_TEMPERATURE, "");
      break;
    case StartupState::INTENSITY:
      ESP_LOGV(TAG, "Querying intensity...");
      this->send_command_(CATEGORY_INTENSITY, "");
      break;
    case StartupState::LED:
      ESP_LOGV(TAG, "Querying LED...");
      this->send_command_(CATEGORY_LED, "");
      break;
    case StartupState::GPS_STATUS:
      ESP_LOGV(TAG, "Querying GPS status...");
      this->send_command_(CATEGORY_DIAGNOSTICS, "GPS");
      break;
    case StartupState::SETTINGS:
      if (this->pending_settings_index_ < this->pending_setting_queries_.size()) {
        TubeClockSetting setting = this->pending_setting_queries_[this->pending_settings_index_];
        ESP_LOGV(TAG, "Querying setting %u...", static_cast<uint8_t>(setting));
        char action[8];
        snprintf(action, sizeof(action), "%u", static_cast<uint8_t>(setting));
        this->send_command_(CATEGORY_SETTINGS, action);
      } else {
        // No settings pending, proceed to alarm slot queries
        this->startup_state_ = StartupState::ALARM_SLOTS;
        ESP_LOGV(TAG, "No settings to query, querying alarm slots...");
        return;
      }
      break;
    case StartupState::ALARM_SLOTS:
      if (this->pending_alarms_index_ < this->pending_alarm_queries_.size()) {
        uint8_t slot = this->pending_alarm_queries_[this->pending_alarms_index_];
        ESP_LOGV(TAG, "Querying alarm slot %u...", slot);
        char action[4];
        snprintf(action, sizeof(action), "%u", slot);
        this->send_command_(CATEGORY_ALARM, action);
      } else {
        // No alarm slots pending, startup complete
        this->startup_state_ = StartupState::COMPLETE;
        ESP_LOGV(TAG, "Startup queries complete");
        this->notify_listeners_();
        return;
      }
      break;
    case StartupState::COMPLETE:
      return;
  }
  this->startup_query_time_ = millis();
  this->startup_waiting_ = true;
}

void TubeClock::advance_startup_(StartupState state) {
  if (this->startup_state_ == state && this->startup_waiting_) {
    this->startup_waiting_ = false;
    this->startup_state_ = static_cast<StartupState>(static_cast<uint8_t>(state) + 1);
  }
}

void TubeClock::request_setting_query(TubeClockSetting setting) {
  for (auto s : this->pending_setting_queries_) {
    if (s == setting) {
      return;  // Already queued
    }
  }
  this->pending_setting_queries_.push_back(setting);
}

// Command implementations
void TubeClock::query_firmware_info() { this->send_command_(CATEGORY_DIAGNOSTICS, "F"); }

void TubeClock::query_gps_status() { this->send_command_(CATEGORY_DIAGNOSTICS, "GPS"); }

void TubeClock::query_page() { this->send_command_(CATEGORY_PAGE, ""); }

void TubeClock::set_page(uint8_t mode, uint8_t submode) {
  if (!this->is_initialized()) {
    return;
  }
  char action[16];
  if (submode != NO_SUBMODE) {
    snprintf(action, sizeof(action), "%uP%u", mode, submode);
  } else {
    snprintf(action, sizeof(action), "%u", mode);
  }
  this->send_command_(CATEGORY_PAGE, action);
}

void TubeClock::query_keys() { this->send_command_(CATEGORY_KEYS, ""); }

void TubeClock::query_adc() { this->send_command_(CATEGORY_HARDWARE, "ADC"); }

void TubeClock::query_hardware() { this->send_command_(CATEGORY_HARDWARE, "CON"); }

void TubeClock::query_hv_power() { this->send_command_(CATEGORY_HARDWARE, "V"); }

void TubeClock::set_hv_power(bool on) {
  if (!this->is_initialized()) {
    return;
  }
  this->hv_power_ = on;
  this->send_command_(CATEGORY_HARDWARE, on ? "VON" : "VOF");
}

void TubeClock::query_time() { this->send_command_(CATEGORY_TIME, ""); }

void TubeClock::set_time(uint8_t hours, uint8_t minutes, uint8_t seconds, uint16_t year, uint8_t month, uint8_t day) {
  if (!this->is_initialized()) {
    return;
  }
  char action[32];
  snprintf(action, sizeof(action), "%02u%02u%02u%04u%02u%02u", hours, minutes, seconds, year, month, day);
  this->send_command_(CATEGORY_TIME, action);
}

void TubeClock::query_temperature() { this->send_command_(CATEGORY_TEMPERATURE, ""); }

void TubeClock::set_temperature(float temp_celsius) {
  if (!this->is_initialized()) {
    return;
  }
  // Convert to tenths of degrees
  int16_t temp_tenths = static_cast<int16_t>(temp_celsius * 10.0f);
  char action[16];
  snprintf(action, sizeof(action), "%d", temp_tenths);
  this->send_command_(CATEGORY_TEMPERATURE, action);
}

void TubeClock::query_temperature_source() { this->send_command_(CATEGORY_TEMPERATURE, "S"); }

void TubeClock::set_temperature_source(TubeClockTempSource source) {
  if (!this->is_initialized()) {
    return;
  }
  char action[8];
  snprintf(action, sizeof(action), "S%u", static_cast<uint8_t>(source));
  this->send_command_(CATEGORY_TEMPERATURE, action);
}

void TubeClock::query_intensity() { this->send_command_(CATEGORY_INTENSITY, ""); }

void TubeClock::set_intensity(uint8_t intensity) {
  if (!this->is_initialized()) {
    return;
  }
  this->intensity_ = intensity;
  char action[8];
  snprintf(action, sizeof(action), "%u", intensity);
  this->send_command_(CATEGORY_INTENSITY, action);
}

void TubeClock::set_auto_intensity(bool enable) {
  if (!this->is_initialized()) {
    return;
  }
  char action[8];
  snprintf(action, sizeof(action), "A%u", enable ? 1 : 0);
  this->send_command_(CATEGORY_INTENSITY, action);
}

void TubeClock::query_led() { this->send_command_(CATEGORY_LED, ""); }

void TubeClock::set_led(uint8_t intensity, uint8_t r, uint8_t g, uint8_t b, bool gamma) {
  if (!this->is_initialized()) {
    return;
  }
  this->led_intensity_ = intensity;
  this->led_r_ = r;
  this->led_g_ = g;
  this->led_b_ = b;
  this->led_gamma_ = gamma;
  char action[32];
  snprintf(action, sizeof(action), "%u,%u,%u,%u,%u", intensity, r, g, b, gamma ? 1 : 0);
  this->send_command_(CATEGORY_LED, action);
}

void TubeClock::set_led_auto(bool enable) {
  if (!this->is_initialized()) {
    return;
  }
  char action[8];
  snprintf(action, sizeof(action), "A%u", enable ? 1 : 0);
  this->send_command_(CATEGORY_LED, action);
}

void TubeClock::query_setting(TubeClockSetting setting) {
  char action[8];
  snprintf(action, sizeof(action), "%u", static_cast<uint8_t>(setting));
  this->send_command_(CATEGORY_SETTINGS, action);
}

void TubeClock::set_setting(TubeClockSetting setting, uint16_t value) {
  if (!this->is_initialized()) {
    return;
  }
  char action[16];
  snprintf(action, sizeof(action), "%u,%u", static_cast<uint8_t>(setting), value);
  this->send_command_(CATEGORY_SETTINGS, action);
}

void TubeClock::save_settings() {
  if (!this->is_initialized()) {
    return;
  }
  this->send_command_(CATEGORY_SETTINGS, "W");
}

void TubeClock::factory_reset() {
  if (!this->is_initialized()) {
    return;
  }
  ESP_LOGW(TAG, "Erasing settings flash (factory reset)");
  this->send_command_(CATEGORY_SETTINGS, "ERASE");
}

void TubeClock::reset() { this->pulse_reset_pin_(); }

void TubeClock::handle_buzzer_status_(const std::string &data) {
  // Boot notification: data = "OOT<version>" (full message was "$TCSBOOT<version>*XX")
  if (data.size() >= 3 && data[0] == 'O' && data[1] == 'O' && data[2] == 'T') {
    this->handle_boot_notification_(data.substr(3));
    return;
  }

  // Responses/notifications:
  //   "P"    -> playback started (response to play command or query)
  //   "S"    -> playback stopped (response to stop command or query)
  //   "OK"   -> playback complete (unsolicited)
  bool new_playing;
  if (data == "P") {
    new_playing = true;
  } else if (data == "S" || data == "OK") {
    new_playing = false;
  } else {
    ESP_LOGV(TAG, "Unknown buzzer status: %s", data.c_str());
    return;
  }

  if (new_playing != this->rtttl_playing_) {
    this->rtttl_playing_ = new_playing;
    ESP_LOGV(TAG, "RTTTL playing: %s", ONOFF(this->rtttl_playing_));
    if (this->is_initialized()) {
      this->notify_listeners_();
    }
  }
}

void TubeClock::handle_boot_notification_(const std::string &data) {
  size_t comma = data.find(',');
  if (comma != std::string::npos) {
    this->firmware_version_ = data.substr(0, comma);
    uint32_t build = 0;
    sscanf(data.c_str() + comma + 1, "%lu", &build);
    this->firmware_build_ = build;
  } else {
    this->firmware_version_ = data;
    this->firmware_build_ = 0;
  }
  ESP_LOGI(TAG, "Clock booted (firmware %s, build %lu), running startup queries", this->firmware_version_.c_str(),
           this->firmware_build_);
  this->reset_startup_();
}

void TubeClock::reset_startup_() {
  this->startup_state_ = StartupState::PAGE;
  this->startup_waiting_ = false;
  this->pending_settings_index_ = 0;
  this->pending_alarms_index_ = 0;
}

void TubeClock::pulse_reset_pin_() {
  if (this->reset_pin_ == nullptr) {
    return;
  }
  ESP_LOGD(TAG, "Asserting hardware reset");
  this->reset_pin_->digital_write(false);
  this->reset_assert_time_ = millis();
  this->reset_asserting_ = true;
}

void TubeClock::play_rtttl(const std::string &rtttl) {
  if (!this->is_initialized()) {
    return;
  }
  // Spec: max RTTTL payload is 251 bytes (255-byte limit minus 3-byte "CBP" prefix and 1-byte null)
  static constexpr size_t MAX_RTTTL_LEN = 251;
  if (rtttl.size() > MAX_RTTTL_LEN) {
    ESP_LOGW(TAG, "RTTTL string too long (%zu bytes), truncating to %zu", rtttl.size(), MAX_RTTTL_LEN);
    this->send_command_(CATEGORY_BUZZER, "P" + rtttl.substr(0, MAX_RTTTL_LEN));
    return;
  }
  this->send_command_(CATEGORY_BUZZER, "P" + rtttl);
}

void TubeClock::stop_rtttl() {
  if (!this->is_initialized()) {
    return;
  }
  this->send_command_(CATEGORY_BUZZER, "S");
}

void TubeClock::play_chime() {
  if (!this->is_initialized()) {
    return;
  }
  this->send_command_(CATEGORY_BUZZER, "C");
}

void TubeClock::play_chime(uint8_t hour) {
  if (!this->is_initialized()) {
    return;
  }
  char action[8];
  snprintf(action, sizeof(action), "C%02u", hour);
  this->send_command_(CATEGORY_BUZZER, action);
}

void TubeClock::query_rtttl_status() { this->send_command_(CATEGORY_BUZZER, "Q"); }

void TubeClock::query_alarm_slot(uint8_t slot) {
  char action[4];
  snprintf(action, sizeof(action), "%u", slot);
  this->send_command_(CATEGORY_ALARM, action);
}

void TubeClock::set_alarm_slot(uint8_t slot, uint8_t hour, uint8_t minute, uint8_t second) {
  if (!this->is_initialized()) {
    return;
  }
  if (slot < 1 || slot > ALARM_SLOT_COUNT) {
    return;
  }
  char action[16];
  snprintf(action, sizeof(action), "%u,%02u%02u%02u", slot, hour, minute, second);
  this->send_command_(CATEGORY_ALARM, action);
}

void TubeClock::request_alarm_query(uint8_t slot) {
  if (slot < 1 || slot > ALARM_SLOT_COUNT) {
    return;
  }
  for (auto s : this->pending_alarm_queries_) {
    if (s == slot) {
      return;  // Already queued
    }
  }
  this->pending_alarm_queries_.push_back(slot);
}

void TubeClock::handle_alarm_status_(const std::string &data) {
  // Format: "<n>,<HHMMSS>"
  size_t comma = data.find(',');
  if (comma == std::string::npos) {
    return;
  }
  uint8_t slot = 0;
  sscanf(data.c_str(), "%hhu", &slot);
  if (slot < 1 || slot > ALARM_SLOT_COUNT) {
    return;
  }
  const char *hhmmss = data.c_str() + comma + 1;
  unsigned int h = 0, m = 0, s = 0;
  if (sscanf(hhmmss, "%2u%2u%2u", &h, &m, &s) != 3) {
    return;
  }
  uint8_t idx = slot - 1;
  uint8_t new_h = static_cast<uint8_t>(h);
  uint8_t new_m = static_cast<uint8_t>(m);
  uint8_t new_s = static_cast<uint8_t>(s);
  if (new_h != this->alarm_hour_[idx] || new_m != this->alarm_minute_[idx] || new_s != this->alarm_second_[idx]) {
    this->alarm_hour_[idx] = new_h;
    this->alarm_minute_[idx] = new_m;
    this->alarm_second_[idx] = new_s;
    ESP_LOGV(TAG, "Alarm slot %u: %02u:%02u:%02u", slot, new_h, new_m, new_s);
    if (this->is_initialized()) {
      this->notify_listeners_();
    }
  }
  if (this->startup_state_ == StartupState::ALARM_SLOTS && this->startup_waiting_) {
    this->startup_waiting_ = false;
    this->pending_alarms_index_++;
    if (this->pending_alarms_index_ >= this->pending_alarm_queries_.size()) {
      this->startup_state_ = StartupState::COMPLETE;
      ESP_LOGV(TAG, "Startup queries complete");
      this->notify_listeners_();
    }
  }
}

void TubeClock::handle_timer_status_(const std::string &data) {
  // Alarm active unsolicited notification: "ALM"
  if (data == "ALM") {
    if (!this->timer_alarm_active_) {
      this->timer_alarm_active_ = true;
      ESP_LOGD(TAG, "Timer alarm active");
      if (this->is_initialized()) {
        this->notify_listeners_();
      }
    }
    return;
  }

  // Clear alarm response: "A<0|1>"
  // 1 = alarm was active and has been cleared; 0 = no alarm was active
  if (data.size() >= 2 && data[0] == 'A') {
    uint8_t cleared = 0;
    sscanf(data.c_str() + 1, "%hhu", &cleared);
    if (cleared == 1 && this->timer_alarm_active_) {
      this->timer_alarm_active_ = false;
      ESP_LOGD(TAG, "Timer alarm cleared");
      if (this->is_initialized()) {
        this->notify_listeners_();
      }
    }
    return;
  }

  // State+value response: "<state>,<value>"
  // state: 'U'=running up, 'D'=running down, 'S'=stopped, 'R'=reset (transient)
  if (data.size() >= 3 && data[1] == ',') {
    char new_state = data[0];
    if (new_state != 'U' && new_state != 'D' && new_state != 'S' && new_state != 'R') {
      ESP_LOGV(TAG, "Unknown timer status: %s", data.c_str());
      return;
    }
    uint32_t new_value = 0;
    sscanf(data.c_str() + 2, "%lu", &new_value);
    if (new_state != this->timer_state_ || new_value != this->timer_value_) {
      this->timer_state_ = new_state;
      this->timer_value_ = new_value;
      ESP_LOGV(TAG, "Timer: state=%c, value=%lu", this->timer_state_, (unsigned long) this->timer_value_);
      if (this->is_initialized()) {
        this->notify_listeners_();
      }
    }
    return;
  }

  ESP_LOGV(TAG, "Unknown timer status: %s", data.c_str());
}

void TubeClock::timer_run_up() {
  if (!this->is_initialized()) {
    return;
  }
  this->send_command_(CATEGORY_TIMER, "U");
}

void TubeClock::timer_run_down() {
  if (!this->is_initialized()) {
    return;
  }
  this->send_command_(CATEGORY_TIMER, "D");
}

void TubeClock::timer_stop() {
  if (!this->is_initialized()) {
    return;
  }
  this->send_command_(CATEGORY_TIMER, "S");
}

void TubeClock::timer_reset() {
  if (!this->is_initialized()) {
    return;
  }
  this->send_command_(CATEGORY_TIMER, "R");
}

void TubeClock::timer_reset_to(uint32_t seconds) {
  if (!this->is_initialized()) {
    return;
  }
  char action[16];
  snprintf(action, sizeof(action), "R%lu", (unsigned long) seconds);
  this->send_command_(CATEGORY_TIMER, action);
}

void TubeClock::timer_clear_alarm() {
  if (!this->is_initialized()) {
    return;
  }
  this->send_command_(CATEGORY_TIMER, "A");
}

void TubeClock::enter_bootloader() {
  if (this->boot_0_pin_ != nullptr) {
    ESP_LOGD(TAG, "Entering bootloader via hardware BOOT0 pin");
    this->boot_0_pin_->digital_write(true);
    this->release_boot0_after_reset_ = true;
    this->pulse_reset_pin_();
    return;
  }
  if (!this->is_initialized()) {
    return;
  }
  ESP_LOGD(TAG, "Entering bootloader");
  this->send_command_(CATEGORY_HARDWARE, "BOOT2");
}

#ifdef USE_TUBE_CLOCK_DFU
bool TubeClock::update_firmware(const std::string &url) {
  if (this->dfu_state_ != DfuState::IDLE) {
    ESP_LOGW(TAG, "Firmware update already in progress");
    return false;
  }
  ESP_LOGD(TAG, "Starting firmware update from %s", url.c_str());
  this->dfu_url_ = url;
  if (this->boot_0_pin_ != nullptr) {
    // Hardware bootloader entry: assert BOOT0 and start the reset pulse non-blocking.
    // loop() will call start_dfu_flash_() once the pulse and delay complete.
    this->boot_0_pin_->digital_write(true);
    this->pulse_reset_pin_();
    this->dfu_state_ = DfuState::AWAITING_RESET_PULSE_MS;
  } else if (this->is_initialized()) {
    this->send_command_(CATEGORY_HARDWARE, "BOOT1");
    this->dfu_state_ = DfuState::AWAITING_ACK;
    this->dfu_ack_deadline_ms_ = millis() + 2000;
  } else {
    ESP_LOGW(TAG, "Clock not initialized, skipping bootloader request");
    this->pulse_reset_pin_();
    this->dfu_state_ = DfuState::AWAITING_RESET_PULSE_MS;
  }
  return true;
}

void TubeClock::restore_dfu_uart_() {
  uart::UARTComponent *dfu_parent = this->parent_;
  ESP_LOGV(TAG, "Restoring primary UART settings");
  dfu_parent->set_baud_rate(this->dfu_original_baud_rate_);
  dfu_parent->set_parity(uart::UART_CONFIG_PARITY_NONE);
  dfu_parent->load_settings(false);
  this->pulse_reset_pin_();
}

void TubeClock::start_dfu_flash_() {
  ESP_LOGD(TAG, "STM32 bootloader ready, starting DFU for %s", this->dfu_url_.c_str());

  uart::UARTComponent *dfu_parent = this->parent_;
  this->dfu_original_baud_rate_ = dfu_parent->get_baud_rate();

  // Reconfigure the primary UART for STM32 bootloader protocol
  if (this->dfu_baud_rate_ != 0) {
    ESP_LOGD(TAG, "Reconfiguring UART baud rate to %" PRIu32 " for DFU", this->dfu_baud_rate_);
    dfu_parent->set_baud_rate(this->dfu_baud_rate_);
  }
  dfu_parent->set_parity(this->dfu_parity_);
  dfu_parent->load_settings(false);
  ESP_LOGD(TAG, "Reconfigured UART for DFU (parity=%s, baud=%" PRIu32 ")",
           this->dfu_parity_ == uart::UART_CONFIG_PARITY_EVEN ? "EVEN" : "NONE", dfu_parent->get_baud_rate());

#ifdef USE_ESP_IDF
  if (this->dfu_tx_pin_ != -1 || this->dfu_rx_pin_ != -1) {
    auto *idf_uart = static_cast<uart::IDFUARTComponent *>(dfu_parent);
    uart_set_pin(static_cast<uart_port_t>(idf_uart->get_hw_serial_number()),
                 this->dfu_tx_pin_ != -1 ? this->dfu_tx_pin_ : UART_PIN_NO_CHANGE,
                 this->dfu_rx_pin_ != -1 ? this->dfu_rx_pin_ : UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE,
                 UART_PIN_NO_CHANGE);
    ESP_LOGD(TAG, "Routed UART to DFU pins (TX=%d, RX=%d)", this->dfu_tx_pin_, this->dfu_rx_pin_);
  }
#endif

  if (this->boot_0_pin_ != nullptr)
    this->boot_0_pin_->digital_write(false);

  this->dfu_device_.set_uart_parent(dfu_parent);

  // Open HTTP connection (brief blocking: TCP handshake + response headers)
  esp_http_client_config_t http_config = {};
  http_config.url = this->dfu_url_.c_str();
  http_config.timeout_ms = 15000;
  http_config.disable_auto_redirect = false;
  http_config.max_redirection_count = 5;

  this->dfu_http_client_ = esp_http_client_init(&http_config);
  if (this->dfu_http_client_ == nullptr) {
    ESP_LOGE(TAG, "Failed to initialize HTTP client");
    this->restore_dfu_uart_();
    this->dfu_state_ = DfuState::IDLE;
    return;
  }

  const esp_err_t http_err = esp_http_client_open(this->dfu_http_client_, 0);
  if (http_err != ESP_OK) {
    ESP_LOGE(TAG, "HTTP open failed: %s", esp_err_to_name(http_err));
    esp_http_client_cleanup(this->dfu_http_client_);
    this->dfu_http_client_ = nullptr;
    this->restore_dfu_uart_();
    this->dfu_state_ = DfuState::IDLE;
    return;
  }

  const int64_t content_length = esp_http_client_fetch_headers(this->dfu_http_client_);
  if (content_length <= 0) {
    ESP_LOGE(TAG, "Invalid content length: %lld", content_length);
    esp_http_client_close(this->dfu_http_client_);
    esp_http_client_cleanup(this->dfu_http_client_);
    this->dfu_http_client_ = nullptr;
    this->restore_dfu_uart_();
    this->dfu_state_ = DfuState::IDLE;
    return;
  }
  ESP_LOGD(TAG, "Firmware size: %lld bytes", content_length);

  // Settings are at the second half of page 63 (0x0801FC00–0x0801FFFF).
  // Erasing only pages 0–62 guarantees settings are preserved.
  static constexpr uint32_t SETTINGS_PAGE = 63;
  this->dfu_flasher_.begin(&this->dfu_device_, static_cast<size_t>(content_length), SETTINGS_PAGE);

  this->dfu_state_ = DfuState::FLASHING;
}

void TubeClock::tick_dfu_() {
  auto result = this->dfu_flasher_.tick();

  if (result == Stm32Flasher::Result::NEEDS_DATA) {
    uint8_t buf[256];
    const int n = esp_http_client_read(this->dfu_http_client_, reinterpret_cast<char *>(buf), sizeof(buf));
    if (n > 0) {
      this->dfu_flasher_.provide_chunk(buf, static_cast<unsigned int>(n));
      result = Stm32Flasher::Result::PENDING;
    } else {
      ESP_LOGE(TAG, "HTTP read failed");
      this->dfu_flasher_.reset();
      result = Stm32Flasher::Result::FAILED;
    }
  }

  if (result == Stm32Flasher::Result::DONE || result == Stm32Flasher::Result::FAILED) {
    if (result == Stm32Flasher::Result::DONE) {
      ESP_LOGI(TAG, "Firmware update complete (%.1f%%)", this->dfu_flasher_.progress() * 100.0f);
    } else {
      ESP_LOGE(TAG, "Firmware update failed");
    }
    esp_http_client_close(this->dfu_http_client_);
    esp_http_client_cleanup(this->dfu_http_client_);
    this->dfu_http_client_ = nullptr;
    this->restore_dfu_uart_();
    this->dfu_state_ = DfuState::IDLE;
  }
}
#endif  // USE_TUBE_CLOCK_DFU

}  // namespace esphome::tube_clock
