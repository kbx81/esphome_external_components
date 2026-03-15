/**
 * @file tube_clock_binary_sensor.cpp
 * @brief Implementation of Tube Clock binary sensor platform
 */

#include "tube_clock_binary_sensor.h"
#include "esphome/core/log.h"

namespace esphome::tube_clock {

static const char *const TAG = "tube_clock.binary_sensor";

void TubeClockBinarySensor::setup() {
  this->parent_->register_listener(this);
  this->on_tube_clock_update();
}

void TubeClockBinarySensor::dump_config() {
  LOG_BINARY_SENSOR("", "Tube Clock Binary Sensor", this);
  switch (this->type_) {
    case BINARY_SENSOR_TYPE_KEYS:
      ESP_LOGCONFIG(TAG, "  Type: Keys (mask=0x%02X)", this->key_mask_);
      break;
    case BINARY_SENSOR_TYPE_GPS_CONNECTED:
      ESP_LOGCONFIG(TAG, "  Type: GpsConnected");
      break;
    case BINARY_SENSOR_TYPE_GPS_FIX_VALID:
      ESP_LOGCONFIG(TAG, "  Type: GpsFixValid");
      break;
    case BINARY_SENSOR_TYPE_RTTTL_PLAYING:
      ESP_LOGCONFIG(TAG, "  Type: RtttlPlaying");
      break;
    case BINARY_SENSOR_TYPE_TIMER_ALARM_ACTIVE:
      ESP_LOGCONFIG(TAG, "  Type: TimerAlarmActive");
      break;
  }
}

void TubeClockBinarySensor::on_tube_clock_update() {
  bool state = false;

  switch (this->type_) {
    case BINARY_SENSOR_TYPE_KEYS:
      // True only when ALL keys in the mask are simultaneously pressed.
      // A zero mask (unconfigured) always reports false.
      state = this->key_mask_ != 0 && (this->parent_->get_key_state() & this->key_mask_) == this->key_mask_;
      break;
    case BINARY_SENSOR_TYPE_GPS_CONNECTED:
      state = this->parent_->get_gps_connected();
      break;
    case BINARY_SENSOR_TYPE_GPS_FIX_VALID:
      state = (this->parent_->get_hardware_mask() & HARDWARE_GPS_FIX_VALID) != 0;
      break;
    case BINARY_SENSOR_TYPE_RTTTL_PLAYING:
      state = this->parent_->get_rtttl_playing();
      break;
    case BINARY_SENSOR_TYPE_TIMER_ALARM_ACTIVE:
      state = this->parent_->get_timer_alarm_active();
      break;
  }

  this->publish_state(state);
}

}  // namespace esphome::tube_clock
