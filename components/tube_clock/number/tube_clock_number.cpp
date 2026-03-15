/**
 * @file tube_clock_number.cpp
 * @brief Implementation of Tube Clock number platform
 */

#include "tube_clock_number.h"
#include "esphome/core/log.h"

namespace esphome::tube_clock {

static const char *const TAG = "tube_clock.number";

static const char *const TYPE_TIME_DISPLAY_DURATION_STR = "TimeDisplayDuration";
static const char *const TYPE_DATE_DISPLAY_DURATION_STR = "DateDisplayDuration";
static const char *const TYPE_TEMP_DISPLAY_DURATION_STR = "TempDisplayDuration";
static const char *const TYPE_FADE_DURATION_STR = "FadeDuration";
static const char *const TYPE_EFFECT_DURATION_STR = "EffectDuration";
static const char *const TYPE_EFFECT_FREQUENCY_STR = "EffectFrequency";
static const char *const TYPE_MINIMUM_INTENSITY_STR = "MinimumIntensity";
static const char *const TYPE_BEEPER_VOLUME_STR = "BeeperVolume";
static const char *const TYPE_TEMP_CAL_STM32_STR = "TempCalStm32";
static const char *const TYPE_TEMP_CAL_DS3234_STR = "TempCalDs3234";
static const char *const TYPE_TEMP_CAL_DS1722_STR = "TempCalDs1722";
static const char *const TYPE_TEMP_CAL_LM74_STR = "TempCalLm74";
static const char *const TYPE_TIMER_RESET_VALUE_STR = "TimerResetValue";
static const char *const TYPE_IDLE_TIMEOUT_STR = "IdleTimeout";

// Temperature calibration raw values span 0–198; subtract this midpoint to get
// the signed offset in tenths of a degree Celsius (range –9.9..+9.9 °C).
static const uint16_t TEMP_CAL_MIDPOINT = 99;

void TubeClockNumber::setup() {
  this->parent_->register_listener(this);
  this->parent_->request_setting_query(this->get_setting_index_());
}

void TubeClockNumber::dump_config() {
  LOG_NUMBER("", "Tube Clock Number", this);
  ESP_LOGCONFIG(TAG, "  Type: %s", this->get_type_as_str());
}

void TubeClockNumber::set_type(const std::string &type) {
  if (type == TYPE_TIME_DISPLAY_DURATION_STR) {
    this->type_ = NUMBER_TYPE_TIME_DISPLAY_DURATION;
  } else if (type == TYPE_DATE_DISPLAY_DURATION_STR) {
    this->type_ = NUMBER_TYPE_DATE_DISPLAY_DURATION;
  } else if (type == TYPE_TEMP_DISPLAY_DURATION_STR) {
    this->type_ = NUMBER_TYPE_TEMP_DISPLAY_DURATION;
  } else if (type == TYPE_FADE_DURATION_STR) {
    this->type_ = NUMBER_TYPE_FADE_DURATION;
  } else if (type == TYPE_EFFECT_DURATION_STR) {
    this->type_ = NUMBER_TYPE_EFFECT_DURATION;
  } else if (type == TYPE_EFFECT_FREQUENCY_STR) {
    this->type_ = NUMBER_TYPE_EFFECT_FREQUENCY;
  } else if (type == TYPE_MINIMUM_INTENSITY_STR) {
    this->type_ = NUMBER_TYPE_MINIMUM_INTENSITY;
  } else if (type == TYPE_BEEPER_VOLUME_STR) {
    this->type_ = NUMBER_TYPE_BEEPER_VOLUME;
  } else if (type == TYPE_TEMP_CAL_STM32_STR) {
    this->type_ = NUMBER_TYPE_TEMP_CAL_STM32;
  } else if (type == TYPE_TEMP_CAL_DS3234_STR) {
    this->type_ = NUMBER_TYPE_TEMP_CAL_DS3234;
  } else if (type == TYPE_TEMP_CAL_DS1722_STR) {
    this->type_ = NUMBER_TYPE_TEMP_CAL_DS1722;
  } else if (type == TYPE_TEMP_CAL_LM74_STR) {
    this->type_ = NUMBER_TYPE_TEMP_CAL_LM74;
  } else if (type == TYPE_TIMER_RESET_VALUE_STR) {
    this->type_ = NUMBER_TYPE_TIMER_RESET_VALUE;
  } else if (type == TYPE_IDLE_TIMEOUT_STR) {
    this->type_ = NUMBER_TYPE_IDLE_TIMEOUT;
  }
}

const char *TubeClockNumber::get_type_as_str() const {
  switch (this->type_) {
    case NUMBER_TYPE_TIME_DISPLAY_DURATION:
      return TYPE_TIME_DISPLAY_DURATION_STR;
    case NUMBER_TYPE_DATE_DISPLAY_DURATION:
      return TYPE_DATE_DISPLAY_DURATION_STR;
    case NUMBER_TYPE_TEMP_DISPLAY_DURATION:
      return TYPE_TEMP_DISPLAY_DURATION_STR;
    case NUMBER_TYPE_FADE_DURATION:
      return TYPE_FADE_DURATION_STR;
    case NUMBER_TYPE_EFFECT_DURATION:
      return TYPE_EFFECT_DURATION_STR;
    case NUMBER_TYPE_EFFECT_FREQUENCY:
      return TYPE_EFFECT_FREQUENCY_STR;
    case NUMBER_TYPE_MINIMUM_INTENSITY:
      return TYPE_MINIMUM_INTENSITY_STR;
    case NUMBER_TYPE_BEEPER_VOLUME:
      return TYPE_BEEPER_VOLUME_STR;
    case NUMBER_TYPE_TEMP_CAL_STM32:
      return TYPE_TEMP_CAL_STM32_STR;
    case NUMBER_TYPE_TEMP_CAL_DS3234:
      return TYPE_TEMP_CAL_DS3234_STR;
    case NUMBER_TYPE_TEMP_CAL_DS1722:
      return TYPE_TEMP_CAL_DS1722_STR;
    case NUMBER_TYPE_TEMP_CAL_LM74:
      return TYPE_TEMP_CAL_LM74_STR;
    case NUMBER_TYPE_TIMER_RESET_VALUE:
      return TYPE_TIMER_RESET_VALUE_STR;
    case NUMBER_TYPE_IDLE_TIMEOUT:
      return TYPE_IDLE_TIMEOUT_STR;
    default:
      return "Unknown";
  }
}

TubeClockSetting TubeClockNumber::get_setting_index_() {
  switch (this->type_) {
    case NUMBER_TYPE_TIME_DISPLAY_DURATION:
      return SETTING_TIME_DISPLAY_DURATION;
    case NUMBER_TYPE_DATE_DISPLAY_DURATION:
      return SETTING_DATE_DISPLAY_DURATION;
    case NUMBER_TYPE_TEMP_DISPLAY_DURATION:
      return SETTING_TEMPERATURE_DISPLAY_DURATION;
    case NUMBER_TYPE_FADE_DURATION:
      return SETTING_FADE_DURATION;
    case NUMBER_TYPE_EFFECT_DURATION:
      return SETTING_EFFECT_DURATION;
    case NUMBER_TYPE_EFFECT_FREQUENCY:
      return SETTING_EFFECT_FREQUENCY;
    case NUMBER_TYPE_MINIMUM_INTENSITY:
      return SETTING_MINIMUM_INTENSITY;
    case NUMBER_TYPE_BEEPER_VOLUME:
      return SETTING_BEEPER_VOLUME;
    case NUMBER_TYPE_TEMP_CAL_STM32:
      return SETTING_TEMP_CALIBRATION_STM32;
    case NUMBER_TYPE_TEMP_CAL_DS3234:
      return SETTING_TEMP_CALIBRATION_DS3234;
    case NUMBER_TYPE_TEMP_CAL_DS1722:
      return SETTING_TEMP_CALIBRATION_DS1722;
    case NUMBER_TYPE_TEMP_CAL_LM74:
      return SETTING_TEMP_CALIBRATION_LM74;
    case NUMBER_TYPE_TIMER_RESET_VALUE:
      return SETTING_TIMER_RESET_VALUE;
    case NUMBER_TYPE_IDLE_TIMEOUT:
      return SETTING_IDLE_TIMEOUT;
    default:
      return SETTING_SYSTEM_OPTIONS;
  }
}

void TubeClockNumber::on_tube_clock_update() {
  TubeClockSetting setting = this->get_setting_index_();
  uint16_t raw = this->parent_->get_setting(setting);
  float new_state;
  if (this->type_ == NUMBER_TYPE_TEMP_CAL_STM32 || this->type_ == NUMBER_TYPE_TEMP_CAL_DS3234 ||
      this->type_ == NUMBER_TYPE_TEMP_CAL_DS1722 || this->type_ == NUMBER_TYPE_TEMP_CAL_LM74) {
    // Raw 0–198 encodes offset in tenths of °C: offset = (raw − 99) / 10
    new_state = (static_cast<float>(raw) - TEMP_CAL_MIDPOINT) / 10.0f;
  } else {
    new_state = static_cast<float>(raw);
  }
  // Only publish if value has changed to reduce log spam
  if (!this->has_state() || this->state != new_state) {
    this->publish_state(new_state);
  }
}

void TubeClockNumber::control(float value) {
  TubeClockSetting setting = this->get_setting_index_();
  uint16_t raw;
  if (this->type_ == NUMBER_TYPE_TEMP_CAL_STM32 || this->type_ == NUMBER_TYPE_TEMP_CAL_DS3234 ||
      this->type_ == NUMBER_TYPE_TEMP_CAL_DS1722 || this->type_ == NUMBER_TYPE_TEMP_CAL_LM74) {
    // Invert: raw = round(offset * 10) + 99
    raw = static_cast<uint16_t>(static_cast<int>(roundf(value * 10.0f)) + TEMP_CAL_MIDPOINT);
  } else {
    raw = static_cast<uint16_t>(value);
  }
  this->parent_->set_setting(setting, raw);
  this->publish_state(value);
}

}  // namespace esphome::tube_clock
