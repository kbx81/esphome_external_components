/**
 * @file tube_clock_select.h
 * @brief Select platform for Tube Clock
 */

#pragma once

#include "esphome/core/component.h"
#include "esphome/components/select/select.h"
#include "../tube_clock.h"

namespace esphome::tube_clock {

enum TubeClockSelectType {
  SELECT_TYPE_OPERATING_MODE,
  SELECT_TYPE_DISPLAY_MODE,
  SELECT_TYPE_TEMPERATURE_SOURCE,
  SELECT_TYPE_COLON_BEHAVIOR,
  SELECT_TYPE_DATE_FORMAT,
  SELECT_TYPE_TIME_DISPLAY_MODE,
  SELECT_TYPE_TEMPERATURE_DISPLAY_UNITS,
};

class TubeClockSelect : public select::Select, public Component, public TubeClockListener {
 public:
  void setup() override;
  void dump_config() override;

  void set_parent(TubeClock *parent) { this->parent_ = parent; }
  void set_type(const std::string &type);

  const char *get_type_as_str() const;

  void on_tube_clock_update() override;

 protected:
  void control(const std::string &value) override;
  std::string get_current_value_();

  TubeClock *parent_{nullptr};
  TubeClockSelectType type_;
};

}  // namespace esphome::tube_clock
