/**
 * @file tube_clock_text_sensor.h
 * @brief Text sensor platform for Tube Clock
 */

#pragma once

#include "esphome/core/component.h"
#include "esphome/components/text_sensor/text_sensor.h"
#include "../tube_clock.h"

namespace esphome {
namespace tube_clock {

enum TubeClockTextSensorType {
  TEXT_SENSOR_TYPE_FIRMWARE_VERSION,
};

class TubeClockTextSensor : public text_sensor::TextSensor, public Component, public TubeClockListener {
 public:
  void setup() override;
  void dump_config() override;

  void set_parent(TubeClock *parent) { this->parent_ = parent; }
  void set_type(TubeClockTextSensorType type) { this->type_ = type; }

  void on_tube_clock_update() override;

 protected:
  TubeClock *parent_{nullptr};
  TubeClockTextSensorType type_{TEXT_SENSOR_TYPE_FIRMWARE_VERSION};
};

}  // namespace tube_clock
}  // namespace esphome
