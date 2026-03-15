/**
 * @file tube_clock_datetime.h
 * @brief Datetime (time) platform for Tube Clock alarm slots
 */

#pragma once

#include "esphome/core/defines.h"

#ifdef USE_DATETIME_TIME

#include "esphome/components/datetime/time_entity.h"
#include "esphome/core/component.h"
#include "../tube_clock.h"

namespace esphome {
namespace tube_clock {

class TubeClockDatetime : public datetime::TimeEntity, public Component, public TubeClockListener {
 public:
  void setup() override;
  void dump_config() override;

  void set_parent(TubeClock *parent) { this->parent_ = parent; }
  void set_slot(uint8_t slot) { this->slot_ = slot; }
  uint8_t get_slot() const { return this->slot_; }

  void on_tube_clock_update() override;

 protected:
  void control(const datetime::TimeCall &call) override;

  TubeClock *parent_{nullptr};
  uint8_t slot_{1};
};

}  // namespace tube_clock
}  // namespace esphome

#endif  // USE_DATETIME_TIME
