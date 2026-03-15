/**
 * @file tube_clock_select.cpp
 * @brief Implementation of Tube Clock select platform
 */

#include "tube_clock_select.h"
#include "esphome/core/log.h"
#include <string>

namespace esphome::tube_clock {

static const char *const TAG = "tube_clock.select";

static const char *const TYPE_OPERATING_MODE_STR = "OperatingMode";
static const char *const TYPE_DISPLAY_MODE_STR = "DisplayMode";
static const char *const TYPE_TEMPERATURE_SOURCE_STR = "TemperatureSource";
static const char *const TYPE_COLON_BEHAVIOR_STR = "ColonBehavior";
static const char *const TYPE_DATE_FORMAT_STR = "DateFormat";
static const char *const TYPE_TIME_DISPLAY_MODE_STR = "TimeDisplayMode";
static const char *const TYPE_TEMPERATURE_DISPLAY_UNITS_STR = "TemperatureDisplayUnits";

// Operating mode options
static constexpr const char *const OPERATING_MODE_OPTIONS[] = {
    "Main Menu",         "Fixed Display",
    "Toggle Display",    "Timer/Counter",
    "DMX512 Display",    "Set Clock",
    "Set Date",          "Set Timer",
    "System Status",     "System Options",
    "Slot Beep",         "Slot Blink",
    "Slot On/Off",       "PM Indicator RGB Config",
    "Duration Clock",    "Duration Date",
    "Duration Temp",     "Duration Fade",
    "DST Begin Month",   "DST Begin Week Ordinal",
    "DST End Month",     "DST End Week Ordinal",
    "DST Switch DOW",    "DST Switch Hour",
    "Effect Duration",   "Effect Frequency",
    "Minimum Intensity", "Beeper Volume",
    "Temp Calibration",  "Idle Timeout",
    "Date Format",       "Time Zone",
    "Colon Behavior",    "DMX512 Address",
    "Slot1 Time",        "Slot2 Time",
    "Slot3 Time",        "Slot4 Time",
    "Slot5 Time",        "Slot6 Time",
    "Slot7 Time",        "Slot8 Time",
};
static constexpr size_t OPERATING_MODE_OPTIONS_SIZE =
    sizeof(OPERATING_MODE_OPTIONS) / sizeof(OPERATING_MODE_OPTIONS[0]);

// Display mode options (view mode / sub-page; meaning is context-dependent on operating mode)
static constexpr const char *const DISPLAY_MODE_OPTIONS[] = {
    "0/Time", "1/Time in Seconds", "2/Date", "3/Temperature", "4", "5", "6", "7", "8", "9",
};
static constexpr size_t DISPLAY_MODE_OPTIONS_SIZE = sizeof(DISPLAY_MODE_OPTIONS) / sizeof(DISPLAY_MODE_OPTIONS[0]);

// Temperature source options
static constexpr const char *const TEMPERATURE_SOURCE_OPTIONS[] = {
    "STM32 Internal", "DS3234 RTC", "DS1722", "LM74", "External Serial",
};
static constexpr size_t TEMPERATURE_SOURCE_OPTIONS_SIZE =
    sizeof(TEMPERATURE_SOURCE_OPTIONS) / sizeof(TEMPERATURE_SOURCE_OPTIONS[0]);

// Colon behavior options
static constexpr const char *const COLON_BEHAVIOR_OPTIONS[] = {
    "On", "Off", "Blink", "Blink Upper", "Blink Lower", "Alternate Upper/Lower",
};
static constexpr size_t COLON_BEHAVIOR_OPTIONS_SIZE =
    sizeof(COLON_BEHAVIOR_OPTIONS) / sizeof(COLON_BEHAVIOR_OPTIONS[0]);

// Date format options (years are 2 digits only)
static constexpr const char *const DATE_FORMAT_OPTIONS[] = {
    "YY-MM-DD",
    "DD-MM-YY",
    "MM-DD-YY",
};
static constexpr size_t DATE_FORMAT_OPTIONS_SIZE = sizeof(DATE_FORMAT_OPTIONS) / sizeof(DATE_FORMAT_OPTIONS[0]);

// Time display mode options
static constexpr const char *const TIME_DISPLAY_MODE_OPTIONS[] = {
    "24-Hour",
    "12-Hour",
};
static constexpr size_t TIME_DISPLAY_MODE_OPTIONS_SIZE =
    sizeof(TIME_DISPLAY_MODE_OPTIONS) / sizeof(TIME_DISPLAY_MODE_OPTIONS[0]);

// Temperature display units options
static constexpr const char *const TEMPERATURE_DISPLAY_UNITS_OPTIONS[] = {
    "Celsius",
    "Fahrenheit",
};
static constexpr size_t TEMPERATURE_DISPLAY_UNITS_OPTIONS_SIZE =
    sizeof(TEMPERATURE_DISPLAY_UNITS_OPTIONS) / sizeof(TEMPERATURE_DISPLAY_UNITS_OPTIONS[0]);

void TubeClockSelect::setup() {
  this->parent_->register_listener(this);

  // Set options based on type - using initializer lists as required by SelectTraits
  switch (this->type_) {
    case SELECT_TYPE_OPERATING_MODE:
      this->traits.set_options({
          "Main Menu",         "Fixed Display",
          "Toggle Display",    "Timer/Counter",
          "DMX512 Display",    "Set Clock",
          "Set Date",          "Set Timer",
          "System Status",     "System Options",
          "Slot Beep",         "Slot Blink",
          "Slot On/Off",       "PM Indicator RGB Config",
          "Duration Clock",    "Duration Date",
          "Duration Temp",     "Duration Fade",
          "DST Begin Month",   "DST Begin Week Ordinal",
          "DST End Month",     "DST End Week Ordinal",
          "DST Switch DOW",    "DST Switch Hour",
          "Effect Duration",   "Effect Frequency",
          "Minimum Intensity", "Beeper Volume",
          "Temp Calibration",  "Idle Timeout",
          "Date Format",       "Time Zone",
          "Colon Behavior",    "DMX512 Address",
          "Slot1 Time",        "Slot2 Time",
          "Slot3 Time",        "Slot4 Time",
          "Slot5 Time",        "Slot6 Time",
          "Slot7 Time",        "Slot8 Time",
      });
      break;
    case SELECT_TYPE_DISPLAY_MODE:
      this->traits.set_options(
          {"0/Time", "1/Time in Seconds", "2/Date", "3/Temperature", "4", "5", "6", "7", "8", "9"});
      break;
    case SELECT_TYPE_TEMPERATURE_SOURCE:
      this->traits.set_options({"STM32 Internal", "DS3234 RTC", "DS1722", "LM74", "External Serial"});
      break;
    case SELECT_TYPE_COLON_BEHAVIOR:
      this->traits.set_options({"On", "Off", "Blink", "Blink Upper", "Blink Lower", "Alternate Upper/Lower"});
      break;
    case SELECT_TYPE_DATE_FORMAT:
      this->traits.set_options({"YY-MM-DD", "DD-MM-YY", "MM-DD-YY"});
      break;
    case SELECT_TYPE_TIME_DISPLAY_MODE:
      this->traits.set_options({"24-Hour", "12-Hour"});
      break;
    case SELECT_TYPE_TEMPERATURE_DISPLAY_UNITS:
      this->traits.set_options({"Celsius", "Fahrenheit"});
      break;
  }

  // Register settings queries for types that depend on a setting value
  switch (this->type_) {
    case SELECT_TYPE_COLON_BEHAVIOR:
      this->parent_->request_setting_query(SETTING_COLON_BEHAVIOR);
      break;
    case SELECT_TYPE_DATE_FORMAT:
      this->parent_->request_setting_query(SETTING_DATE_FORMAT);
      break;
    case SELECT_TYPE_TIME_DISPLAY_MODE:
    case SELECT_TYPE_TEMPERATURE_DISPLAY_UNITS:
      this->parent_->request_setting_query(SETTING_SYSTEM_OPTIONS);
      break;
    default:
      break;
  }

  // Publish initial state
  this->on_tube_clock_update();
}

void TubeClockSelect::dump_config() {
  LOG_SELECT("", "Tube Clock Select", this);
  ESP_LOGCONFIG(TAG, "  Type: %s", this->get_type_as_str());
}

void TubeClockSelect::set_type(const std::string &type) {
  if (type == TYPE_OPERATING_MODE_STR) {
    this->type_ = SELECT_TYPE_OPERATING_MODE;
  } else if (type == TYPE_DISPLAY_MODE_STR) {
    this->type_ = SELECT_TYPE_DISPLAY_MODE;
  } else if (type == TYPE_TEMPERATURE_SOURCE_STR) {
    this->type_ = SELECT_TYPE_TEMPERATURE_SOURCE;
  } else if (type == TYPE_COLON_BEHAVIOR_STR) {
    this->type_ = SELECT_TYPE_COLON_BEHAVIOR;
  } else if (type == TYPE_DATE_FORMAT_STR) {
    this->type_ = SELECT_TYPE_DATE_FORMAT;
  } else if (type == TYPE_TIME_DISPLAY_MODE_STR) {
    this->type_ = SELECT_TYPE_TIME_DISPLAY_MODE;
  } else if (type == TYPE_TEMPERATURE_DISPLAY_UNITS_STR) {
    this->type_ = SELECT_TYPE_TEMPERATURE_DISPLAY_UNITS;
  }
}

const char *TubeClockSelect::get_type_as_str() const {
  switch (this->type_) {
    case SELECT_TYPE_OPERATING_MODE:
      return TYPE_OPERATING_MODE_STR;
    case SELECT_TYPE_DISPLAY_MODE:
      return TYPE_DISPLAY_MODE_STR;
    case SELECT_TYPE_TEMPERATURE_SOURCE:
      return TYPE_TEMPERATURE_SOURCE_STR;
    case SELECT_TYPE_COLON_BEHAVIOR:
      return TYPE_COLON_BEHAVIOR_STR;
    case SELECT_TYPE_DATE_FORMAT:
      return TYPE_DATE_FORMAT_STR;
    case SELECT_TYPE_TIME_DISPLAY_MODE:
      return TYPE_TIME_DISPLAY_MODE_STR;
    case SELECT_TYPE_TEMPERATURE_DISPLAY_UNITS:
      return TYPE_TEMPERATURE_DISPLAY_UNITS_STR;
    default:
      return "Unknown";
  }
}

void TubeClockSelect::on_tube_clock_update() {
  std::string current = this->get_current_value_();
  // Only publish if value has changed to reduce log spam
  if (!current.empty() && (!this->has_state() || this->current_option().str() != current)) {
    this->publish_state(current);
  }
}

std::string TubeClockSelect::get_current_value_() {
  switch (this->type_) {
    case SELECT_TYPE_OPERATING_MODE: {
      uint8_t mode = this->parent_->get_mode();
      if (mode < OPERATING_MODE_OPTIONS_SIZE) {
        return std::string(OPERATING_MODE_OPTIONS[mode]);
      }
      break;
    }
    case SELECT_TYPE_DISPLAY_MODE: {
      uint8_t submode = this->parent_->get_submode();
      if (submode < DISPLAY_MODE_OPTIONS_SIZE) {
        return std::string(DISPLAY_MODE_OPTIONS[submode]);
      }
      return std::string(DISPLAY_MODE_OPTIONS[0]);
    }
    case SELECT_TYPE_TEMPERATURE_SOURCE: {
      uint8_t source = static_cast<uint8_t>(this->parent_->get_temp_source());
      if (source < TEMPERATURE_SOURCE_OPTIONS_SIZE) {
        return std::string(TEMPERATURE_SOURCE_OPTIONS[source]);
      }
      break;
    }
    case SELECT_TYPE_COLON_BEHAVIOR: {
      uint16_t value = this->parent_->get_setting(SETTING_COLON_BEHAVIOR);
      if (value < COLON_BEHAVIOR_OPTIONS_SIZE) {
        return std::string(COLON_BEHAVIOR_OPTIONS[value]);
      }
      break;
    }
    case SELECT_TYPE_DATE_FORMAT: {
      uint16_t value = this->parent_->get_setting(SETTING_DATE_FORMAT);
      if (value < DATE_FORMAT_OPTIONS_SIZE) {
        return std::string(DATE_FORMAT_OPTIONS[value]);
      }
      break;
    }
    case SELECT_TYPE_TIME_DISPLAY_MODE: {
      uint16_t options = this->parent_->get_setting(SETTING_SYSTEM_OPTIONS);
      return std::string(TIME_DISPLAY_MODE_OPTIONS[(options & OPTION_DISPLAY_12_HOUR) ? 1 : 0]);
    }
    case SELECT_TYPE_TEMPERATURE_DISPLAY_UNITS: {
      uint16_t options = this->parent_->get_setting(SETTING_SYSTEM_OPTIONS);
      return std::string(TEMPERATURE_DISPLAY_UNITS_OPTIONS[(options & OPTION_DISPLAY_FAHRENHEIT) ? 1 : 0]);
    }
  }
  return "";
}

void TubeClockSelect::control(const std::string &value) {
  switch (this->type_) {
    case SELECT_TYPE_OPERATING_MODE: {
      // Find index of selected mode
      for (size_t i = 0; i < OPERATING_MODE_OPTIONS_SIZE; i++) {
        if (std::string(OPERATING_MODE_OPTIONS[i]) == value) {
          this->parent_->set_page(i);
          break;
        }
      }
      break;
    }
    case SELECT_TYPE_DISPLAY_MODE: {
      uint8_t current_mode = this->parent_->get_mode();
      for (size_t i = 0; i < DISPLAY_MODE_OPTIONS_SIZE; i++) {
        if (std::string(DISPLAY_MODE_OPTIONS[i]) == value) {
          this->parent_->set_page(current_mode, i);
          break;
        }
      }
      break;
    }
    case SELECT_TYPE_TEMPERATURE_SOURCE: {
      // Find index of selected source
      for (size_t i = 0; i < TEMPERATURE_SOURCE_OPTIONS_SIZE; i++) {
        if (std::string(TEMPERATURE_SOURCE_OPTIONS[i]) == value) {
          this->parent_->set_temperature_source(static_cast<TubeClockTempSource>(i));
          break;
        }
      }
      break;
    }
    case SELECT_TYPE_COLON_BEHAVIOR: {
      // Find index and set setting
      for (size_t i = 0; i < COLON_BEHAVIOR_OPTIONS_SIZE; i++) {
        if (std::string(COLON_BEHAVIOR_OPTIONS[i]) == value) {
          this->parent_->set_setting(SETTING_COLON_BEHAVIOR, i);
          break;
        }
      }
      break;
    }
    case SELECT_TYPE_DATE_FORMAT: {
      // Find index and set setting
      for (size_t i = 0; i < DATE_FORMAT_OPTIONS_SIZE; i++) {
        if (std::string(DATE_FORMAT_OPTIONS[i]) == value) {
          this->parent_->set_setting(SETTING_DATE_FORMAT, i);
          break;
        }
      }
      break;
    }
    case SELECT_TYPE_TIME_DISPLAY_MODE: {
      // Toggle bit in system options
      uint16_t options = this->parent_->get_setting(SETTING_SYSTEM_OPTIONS);
      if (value == "12-Hour") {
        options |= OPTION_DISPLAY_12_HOUR;
      } else {
        options &= ~OPTION_DISPLAY_12_HOUR;
      }
      this->parent_->set_setting(SETTING_SYSTEM_OPTIONS, options);
      break;
    }
    case SELECT_TYPE_TEMPERATURE_DISPLAY_UNITS: {
      // Toggle bit in system options
      uint16_t options = this->parent_->get_setting(SETTING_SYSTEM_OPTIONS);
      if (value == "Fahrenheit") {
        options |= OPTION_DISPLAY_FAHRENHEIT;
      } else {
        options &= ~OPTION_DISPLAY_FAHRENHEIT;
      }
      this->parent_->set_setting(SETTING_SYSTEM_OPTIONS, options);
      break;
    }
  }
}

}  // namespace esphome::tube_clock
