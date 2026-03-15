/**
 * @file tube_clock_sensor.h
 * @brief Sensor platform for Tube Clock
 */

#pragma once

#include "esphome/core/component.h"
#include "esphome/components/sensor/sensor.h"
#include "../tube_clock.h"

namespace esphome {
namespace tube_clock {

enum TubeClockSensorType {
  SENSOR_TYPE_ADC_LIGHT,
  SENSOR_TYPE_ADC_VDDA,
  SENSOR_TYPE_ADC_VBATT,
  SENSOR_TYPE_GPS_SATS,
  SENSOR_TYPE_TEMP_STM32,
  SENSOR_TYPE_TEMP_DS3234,
  SENSOR_TYPE_TEMP_DS1722,
  SENSOR_TYPE_TEMP_LM74,
};

class TubeClockSensor : public sensor::Sensor, public Component, public TubeClockListener {
 public:
  void setup() override;
  void dump_config() override;

  void set_parent(TubeClock *parent) { this->parent_ = parent; }
  void set_type(const std::string &type);

  const char *get_type_as_str() const;

  void on_tube_clock_update() override;

 protected:
  TubeClock *parent_{nullptr};
  TubeClockSensorType type_;
};

}  // namespace tube_clock
}  // namespace esphome
