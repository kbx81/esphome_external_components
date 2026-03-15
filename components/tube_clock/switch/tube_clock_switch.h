/**
 * @file tube_clock_switch.h
 * @brief Switch platform for Tube Clock
 */

#pragma once

#include "esphome/core/component.h"
#include "esphome/components/switch/switch.h"
#include "../tube_clock.h"

namespace esphome {
namespace tube_clock {

enum TubeClockSwitchType {
  SWITCH_TYPE_HV_POWER,
  SWITCH_TYPE_AUTO_BRIGHTNESS_NIXIE,
  SWITCH_TYPE_AUTO_BRIGHTNESS_RGB,
  SWITCH_TYPE_HOURLY_CHIME,
  SWITCH_TYPE_SHOW_LEADING_ZEROS,
  SWITCH_TYPE_STATUS_LED_AS_AM_PM,
  SWITCH_TYPE_ALARM_BEEP,
  SWITCH_TYPE_ALARM_BLINK,
};

class TubeClockSwitch : public switch_::Switch, public Component, public TubeClockListener {
 public:
  void setup() override;
  void dump_config() override;

  void set_parent(TubeClock *parent) { this->parent_ = parent; }
  void set_type(const std::string &type);
  void set_slot(uint8_t slot) { this->slot_ = slot; }

  const char *get_type_as_str() const;

  void on_tube_clock_update() override;

 protected:
  void write_state(bool state) override;

  TubeClock *parent_{nullptr};
  TubeClockSwitchType type_;
  uint8_t slot_{1};
};

}  // namespace tube_clock
}  // namespace esphome
