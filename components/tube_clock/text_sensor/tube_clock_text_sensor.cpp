/**
 * @file tube_clock_text_sensor.cpp
 * @brief Implementation of Tube Clock text sensor platform
 */

#include "tube_clock_text_sensor.h"
#include "esphome/core/log.h"

namespace esphome {
namespace tube_clock {

static const char *const TAG = "tube_clock.text_sensor";

void TubeClockTextSensor::setup() {
  this->parent_->register_listener(this);
  this->on_tube_clock_update();
}

void TubeClockTextSensor::dump_config() { LOG_TEXT_SENSOR("", "Tube Clock Text Sensor", this); }

void TubeClockTextSensor::on_tube_clock_update() {
  switch (this->type_) {
    case TEXT_SENSOR_TYPE_FIRMWARE_VERSION: {
      const std::string &version = this->parent_->get_firmware_version();
      if (version.empty()) {
        break;
      }
      char buf[32];
      snprintf(buf, sizeof(buf), "%s (build %lu)", version.c_str(), this->parent_->get_firmware_build());
      std::string formatted(buf);
      if (formatted != this->state) {
        this->publish_state(formatted);
      }
      break;
    }
  }
}

}  // namespace tube_clock
}  // namespace esphome
