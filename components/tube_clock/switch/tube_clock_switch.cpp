/**
 * @file tube_clock_switch.cpp
 * @brief Implementation of Tube Clock switch platform
 */

#include "tube_clock_switch.h"
#include "esphome/core/log.h"

namespace esphome {
namespace tube_clock {

static const char *const TAG = "tube_clock.switch";

static const char *const TYPE_HV_POWER_STR = "HvPower";
static const char *const TYPE_AUTO_BRIGHTNESS_NIXIE_STR = "AutoBrightnessNixie";
static const char *const TYPE_AUTO_BRIGHTNESS_RGB_STR = "AutoBrightnessRgb";
static const char *const TYPE_HOURLY_CHIME_STR = "HourlyChime";
static const char *const TYPE_SHOW_LEADING_ZEROS_STR = "ShowLeadingZeros";
static const char *const TYPE_STATUS_LED_AS_AM_PM_STR = "StatusLedAsAmPm";
static const char *const TYPE_ALARM_BEEP_STR = "AlarmBeep";
static const char *const TYPE_ALARM_BLINK_STR = "AlarmBlink";

void TubeClockSwitch::setup() {
  this->parent_->register_listener(this);
  if (this->type_ == SWITCH_TYPE_HOURLY_CHIME || this->type_ == SWITCH_TYPE_SHOW_LEADING_ZEROS ||
      this->type_ == SWITCH_TYPE_STATUS_LED_AS_AM_PM) {
    this->parent_->request_setting_query(SETTING_SYSTEM_OPTIONS);
  } else if (this->type_ == SWITCH_TYPE_ALARM_BEEP) {
    this->parent_->request_setting_query(SETTING_BEEP_STATES);
  } else if (this->type_ == SWITCH_TYPE_ALARM_BLINK) {
    this->parent_->request_setting_query(SETTING_BLINK_STATES);
  }
}


void TubeClockSwitch::dump_config() {
  LOG_SWITCH("", "Tube Clock Switch", this);
  ESP_LOGCONFIG(TAG, "  Type: %s", this->get_type_as_str());
  if (this->type_ == SWITCH_TYPE_ALARM_BEEP || this->type_ == SWITCH_TYPE_ALARM_BLINK) {
    ESP_LOGCONFIG(TAG, "  Slot: %u", this->slot_);
  }
}

void TubeClockSwitch::set_type(const std::string &type) {
  if (type == TYPE_HV_POWER_STR) {
    this->type_ = SWITCH_TYPE_HV_POWER;
  } else if (type == TYPE_AUTO_BRIGHTNESS_NIXIE_STR) {
    this->type_ = SWITCH_TYPE_AUTO_BRIGHTNESS_NIXIE;
  } else if (type == TYPE_AUTO_BRIGHTNESS_RGB_STR) {
    this->type_ = SWITCH_TYPE_AUTO_BRIGHTNESS_RGB;
  } else if (type == TYPE_HOURLY_CHIME_STR) {
    this->type_ = SWITCH_TYPE_HOURLY_CHIME;
  } else if (type == TYPE_SHOW_LEADING_ZEROS_STR) {
    this->type_ = SWITCH_TYPE_SHOW_LEADING_ZEROS;
  } else if (type == TYPE_STATUS_LED_AS_AM_PM_STR) {
    this->type_ = SWITCH_TYPE_STATUS_LED_AS_AM_PM;
  } else if (type == TYPE_ALARM_BEEP_STR) {
    this->type_ = SWITCH_TYPE_ALARM_BEEP;
  } else if (type == TYPE_ALARM_BLINK_STR) {
    this->type_ = SWITCH_TYPE_ALARM_BLINK;
  }
}

const char *TubeClockSwitch::get_type_as_str() const {
  switch (this->type_) {
    case SWITCH_TYPE_HV_POWER:
      return TYPE_HV_POWER_STR;
    case SWITCH_TYPE_AUTO_BRIGHTNESS_NIXIE:
      return TYPE_AUTO_BRIGHTNESS_NIXIE_STR;
    case SWITCH_TYPE_AUTO_BRIGHTNESS_RGB:
      return TYPE_AUTO_BRIGHTNESS_RGB_STR;
    case SWITCH_TYPE_HOURLY_CHIME:
      return TYPE_HOURLY_CHIME_STR;
    case SWITCH_TYPE_SHOW_LEADING_ZEROS:
      return TYPE_SHOW_LEADING_ZEROS_STR;
    case SWITCH_TYPE_STATUS_LED_AS_AM_PM:
      return TYPE_STATUS_LED_AS_AM_PM_STR;
    case SWITCH_TYPE_ALARM_BEEP:
      return TYPE_ALARM_BEEP_STR;
    case SWITCH_TYPE_ALARM_BLINK:
      return TYPE_ALARM_BLINK_STR;
    default:
      return "Unknown";
  }
}

void TubeClockSwitch::on_tube_clock_update() {
  bool new_state = false;

  switch (this->type_) {
    case SWITCH_TYPE_HV_POWER:
      new_state = this->parent_->get_hv_power();
      break;
    case SWITCH_TYPE_AUTO_BRIGHTNESS_NIXIE:
      new_state = this->parent_->get_auto_intensity();
      break;
    case SWITCH_TYPE_AUTO_BRIGHTNESS_RGB:
      new_state = this->parent_->get_led_auto();
      break;
    case SWITCH_TYPE_HOURLY_CHIME: {
      uint16_t options = this->parent_->get_setting(SETTING_SYSTEM_OPTIONS);
      new_state = (options & OPTION_HOURLY_CHIME) != 0;
      break;
    }
    case SWITCH_TYPE_SHOW_LEADING_ZEROS: {
      uint16_t options = this->parent_->get_setting(SETTING_SYSTEM_OPTIONS);
      new_state = (options & OPTION_MSDS_OFF) == 0;
      break;
    }
    case SWITCH_TYPE_STATUS_LED_AS_AM_PM: {
      uint16_t options = this->parent_->get_setting(SETTING_SYSTEM_OPTIONS);
      new_state = (options & OPTION_STATUS_LED_AS_AM_PM) != 0;
      break;
    }
    case SWITCH_TYPE_ALARM_BEEP: {
      uint16_t beep_states = this->parent_->get_setting(SETTING_BEEP_STATES);
      new_state = (beep_states & (1u << (this->slot_ - 1))) != 0;
      break;
    }
    case SWITCH_TYPE_ALARM_BLINK: {
      uint16_t blink_states = this->parent_->get_setting(SETTING_BLINK_STATES);
      new_state = (blink_states & (1u << (this->slot_ - 1))) != 0;
      break;
    }
  }

  // Only publish if value has changed to reduce log spam
  if (!this->has_state() || this->state != new_state) {
    this->publish_state(new_state);
  }
}

void TubeClockSwitch::write_state(bool state) {
  // Block all hardware commands (and the resulting publish_state) until startup
  // queries have completed. Without this guard, ESPHome's default-OFF initial
  // state would be published to HA before the clock reports its true state.
  if (!this->parent_->is_initialized())
    return;
  switch (this->type_) {
    case SWITCH_TYPE_HV_POWER:
      this->parent_->set_hv_power(state);
      break;
    case SWITCH_TYPE_AUTO_BRIGHTNESS_NIXIE:
      this->parent_->set_auto_intensity(state);
      break;
    case SWITCH_TYPE_AUTO_BRIGHTNESS_RGB:
      this->parent_->set_led_auto(state);
      break;
    case SWITCH_TYPE_HOURLY_CHIME: {
      uint16_t options = this->parent_->get_setting(SETTING_SYSTEM_OPTIONS);
      if (state) {
        options |= OPTION_HOURLY_CHIME;
      } else {
        options &= ~OPTION_HOURLY_CHIME;
      }
      this->parent_->set_setting(SETTING_SYSTEM_OPTIONS, options);
      break;
    }
    case SWITCH_TYPE_SHOW_LEADING_ZEROS: {
      uint16_t options = this->parent_->get_setting(SETTING_SYSTEM_OPTIONS);
      if (state) {
        options &= ~OPTION_MSDS_OFF;
      } else {
        options |= OPTION_MSDS_OFF;
      }
      this->parent_->set_setting(SETTING_SYSTEM_OPTIONS, options);
      break;
    }
    case SWITCH_TYPE_STATUS_LED_AS_AM_PM: {
      uint16_t options = this->parent_->get_setting(SETTING_SYSTEM_OPTIONS);
      if (state) {
        options |= OPTION_STATUS_LED_AS_AM_PM;
      } else {
        options &= ~OPTION_STATUS_LED_AS_AM_PM;
      }
      this->parent_->set_setting(SETTING_SYSTEM_OPTIONS, options);
      break;
    }
    case SWITCH_TYPE_ALARM_BEEP: {
      uint16_t beep_states = this->parent_->get_setting(SETTING_BEEP_STATES);
      if (state) {
        beep_states |= static_cast<uint16_t>(1u << (this->slot_ - 1));
      } else {
        beep_states &= static_cast<uint16_t>(~(1u << (this->slot_ - 1)));
      }
      this->parent_->set_setting(SETTING_BEEP_STATES, beep_states);
      break;
    }
    case SWITCH_TYPE_ALARM_BLINK: {
      uint16_t blink_states = this->parent_->get_setting(SETTING_BLINK_STATES);
      if (state) {
        blink_states |= static_cast<uint16_t>(1u << (this->slot_ - 1));
      } else {
        blink_states &= static_cast<uint16_t>(~(1u << (this->slot_ - 1)));
      }
      this->parent_->set_setting(SETTING_BLINK_STATES, blink_states);
      break;
    }
  }

  this->publish_state(state);
}

}  // namespace tube_clock
}  // namespace esphome
