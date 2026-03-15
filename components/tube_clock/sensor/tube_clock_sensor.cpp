/**
 * @file tube_clock_sensor.cpp
 * @brief Implementation of Tube Clock sensor platform
 */

#include "tube_clock_sensor.h"
#include "esphome/core/log.h"

namespace esphome {
namespace tube_clock {

static const char *const TAG = "tube_clock.sensor";

static const char *const TYPE_ADC_LIGHT_STR = "AdcLight";
static const char *const TYPE_ADC_VDDA_STR = "AdcVdda";
static const char *const TYPE_ADC_VBATT_STR = "AdcVbatt";
static const char *const TYPE_GPS_SATS_STR = "GpsSats";
static const char *const TYPE_TEMP_STM32_STR = "TempStm32";
static const char *const TYPE_TEMP_DS3234_STR = "TempDs3234";
static const char *const TYPE_TEMP_DS1722_STR = "TempDs1722";
static const char *const TYPE_TEMP_LM74_STR = "TempLm74";

void TubeClockSensor::setup() {
  this->parent_->register_listener(this);
  this->on_tube_clock_update();
}

void TubeClockSensor::dump_config() {
  LOG_SENSOR("", "Tube Clock Sensor", this);
  ESP_LOGCONFIG(TAG, "  Type: %d", this->type_);
}

void TubeClockSensor::set_type(const std::string &type) {
  if (type == TYPE_ADC_LIGHT_STR) {
    this->type_ = SENSOR_TYPE_ADC_LIGHT;
  } else if (type == TYPE_ADC_VDDA_STR) {
    this->type_ = SENSOR_TYPE_ADC_VDDA;
  } else if (type == TYPE_ADC_VBATT_STR) {
    this->type_ = SENSOR_TYPE_ADC_VBATT;
  } else if (type == TYPE_GPS_SATS_STR) {
    this->type_ = SENSOR_TYPE_GPS_SATS;
  } else if (type == TYPE_TEMP_STM32_STR) {
    this->type_ = SENSOR_TYPE_TEMP_STM32;
  } else if (type == TYPE_TEMP_DS3234_STR) {
    this->type_ = SENSOR_TYPE_TEMP_DS3234;
  } else if (type == TYPE_TEMP_DS1722_STR) {
    this->type_ = SENSOR_TYPE_TEMP_DS1722;
  } else if (type == TYPE_TEMP_LM74_STR) {
    this->type_ = SENSOR_TYPE_TEMP_LM74;
  }
}

const char *TubeClockSensor::get_type_as_str() const {
  switch (this->type_) {
    case SENSOR_TYPE_ADC_LIGHT:
      return TYPE_ADC_LIGHT_STR;
    case SENSOR_TYPE_ADC_VDDA:
      return TYPE_ADC_VDDA_STR;
    case SENSOR_TYPE_ADC_VBATT:
      return TYPE_ADC_VBATT_STR;
    case SENSOR_TYPE_GPS_SATS:
      return TYPE_GPS_SATS_STR;
    case SENSOR_TYPE_TEMP_STM32:
      return TYPE_TEMP_STM32_STR;
    case SENSOR_TYPE_TEMP_DS3234:
      return TYPE_TEMP_DS3234_STR;
    case SENSOR_TYPE_TEMP_DS1722:
      return TYPE_TEMP_DS1722_STR;
    case SENSOR_TYPE_TEMP_LM74:
      return TYPE_TEMP_LM74_STR;
    default:
      return "Unknown";
  }
}

void TubeClockSensor::on_tube_clock_update() {
  float value = NAN;

  switch (this->type_) {
    case SENSOR_TYPE_ADC_LIGHT:
      value = this->parent_->get_adc_light();
      break;
    case SENSOR_TYPE_ADC_VDDA:
      // Convert millivolts to volts
      value = this->parent_->get_adc_vdda() / 1000.0f;
      break;
    case SENSOR_TYPE_ADC_VBATT:
      // Convert millivolts to volts
      value = this->parent_->get_adc_vbatt() / 1000.0f;
      break;
    case SENSOR_TYPE_GPS_SATS:
      value = this->parent_->get_gps_sats();
      break;
    case SENSOR_TYPE_TEMP_STM32: {
      int16_t temp = this->parent_->get_temp_stm32();
      if (temp != TEMP_UNAVAILABLE) {
        // Convert tenths of degrees to degrees
        value = temp / 10.0f;
      }
      break;
    }
    case SENSOR_TYPE_TEMP_DS3234: {
      int16_t temp = this->parent_->get_temp_ds3234();
      if (temp != TEMP_UNAVAILABLE) {
        value = temp / 10.0f;
      }
      break;
    }
    case SENSOR_TYPE_TEMP_DS1722: {
      int16_t temp = this->parent_->get_temp_ds1722();
      if (temp != TEMP_UNAVAILABLE) {
        value = temp / 10.0f;
      }
      break;
    }
    case SENSOR_TYPE_TEMP_LM74: {
      int16_t temp = this->parent_->get_temp_lm74();
      if (temp != TEMP_UNAVAILABLE) {
        value = temp / 10.0f;
      }
      break;
    }
  }

  // Only publish if value has changed to reduce log spam
  if (!std::isnan(value) && (std::isnan(this->raw_state) || this->raw_state != value)) {
    this->publish_state(value);
  }
}

}  // namespace tube_clock
}  // namespace esphome
