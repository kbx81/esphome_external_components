/**
 * @file tube_clock_number.h
 * @brief Number platform for Tube Clock
 */

#pragma once

#include "esphome/core/component.h"
#include "esphome/components/number/number.h"
#include "../tube_clock.h"

namespace esphome::tube_clock {

enum TubeClockNumberType {
  NUMBER_TYPE_TIME_DISPLAY_DURATION,
  NUMBER_TYPE_DATE_DISPLAY_DURATION,
  NUMBER_TYPE_TEMP_DISPLAY_DURATION,
  NUMBER_TYPE_FADE_DURATION,
  NUMBER_TYPE_EFFECT_DURATION,
  NUMBER_TYPE_EFFECT_FREQUENCY,
  NUMBER_TYPE_MINIMUM_INTENSITY,
  NUMBER_TYPE_BEEPER_VOLUME,
  NUMBER_TYPE_TEMP_CAL_STM32,
  NUMBER_TYPE_TEMP_CAL_DS3234,
  NUMBER_TYPE_TEMP_CAL_DS1722,
  NUMBER_TYPE_TEMP_CAL_LM74,
  NUMBER_TYPE_TIMER_RESET_VALUE,
  NUMBER_TYPE_IDLE_TIMEOUT,
};

class TubeClockNumber : public number::Number, public Component, public TubeClockListener {
 public:
  void setup() override;
  void dump_config() override;

  void set_parent(TubeClock *parent) { this->parent_ = parent; }
  void set_type(const std::string &type);

  const char *get_type_as_str() const;

  void on_tube_clock_update() override;

 protected:
  void control(float value) override;
  TubeClockSetting get_setting_index_();

  TubeClock *parent_{nullptr};
  TubeClockNumberType type_;
};

}  // namespace esphome::tube_clock
