/**
 * @file tube_clock_binary_sensor.h
 * @brief Binary sensor platform for Tube Clock
 */

#pragma once

#include "esphome/core/component.h"
#include "esphome/components/binary_sensor/binary_sensor.h"
#include "../tube_clock.h"

namespace esphome::tube_clock {

enum TubeClockBinarySensorType {
  BINARY_SENSOR_TYPE_KEYS,
  BINARY_SENSOR_TYPE_GPS_CONNECTED,
  BINARY_SENSOR_TYPE_GPS_FIX_VALID,
  BINARY_SENSOR_TYPE_RTTTL_PLAYING,
  BINARY_SENSOR_TYPE_TIMER_ALARM_ACTIVE,
};

class TubeClockBinarySensor : public binary_sensor::BinarySensor, public Component, public TubeClockListener {
 public:
  void setup() override;
  void dump_config() override;

  void set_parent(TubeClock *parent) { this->parent_ = parent; }

  // Key combination sensor: true when ALL keys in the mask are simultaneously pressed.
  // key_mask is the OR of TubeClockKey bitmask values for each key to watch.
  void set_key_mask(uint8_t mask) {
    this->type_ = BINARY_SENSOR_TYPE_KEYS;
    this->key_mask_ = mask;
  }

  void set_gps_connected() { this->type_ = BINARY_SENSOR_TYPE_GPS_CONNECTED; }
  void set_gps_fix_valid() { this->type_ = BINARY_SENSOR_TYPE_GPS_FIX_VALID; }
  void set_rtttl_playing() { this->type_ = BINARY_SENSOR_TYPE_RTTTL_PLAYING; }
  void set_timer_alarm_active() { this->type_ = BINARY_SENSOR_TYPE_TIMER_ALARM_ACTIVE; }

  void on_tube_clock_update() override;

 protected:
  TubeClock *parent_{nullptr};
  TubeClockBinarySensorType type_{BINARY_SENSOR_TYPE_KEYS};
  uint8_t key_mask_{0};
};

}  // namespace esphome::tube_clock
