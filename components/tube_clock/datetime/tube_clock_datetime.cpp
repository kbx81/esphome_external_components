/**
 * @file tube_clock_datetime.cpp
 * @brief Implementation of Tube Clock datetime (alarm slot time) platform
 */

#include "tube_clock_datetime.h"

#ifdef USE_DATETIME_TIME

#include "esphome/core/log.h"

namespace esphome {
namespace tube_clock {

static const char *const TAG = "tube_clock.datetime";

void TubeClockDatetime::setup() {
  this->parent_->register_listener(this);
  this->parent_->request_alarm_query(this->slot_);
}

void TubeClockDatetime::dump_config() {
  LOG_DATETIME_TIME("", "Tube Clock Alarm Time", this);
  ESP_LOGCONFIG(TAG, "  Slot: %u", this->slot_);
}

void TubeClockDatetime::on_tube_clock_update() {
  if (!this->parent_->is_initialized())
    return;

  uint8_t h = this->parent_->get_alarm_hour(this->slot_);
  uint8_t m = this->parent_->get_alarm_minute(this->slot_);
  uint8_t s = this->parent_->get_alarm_second(this->slot_);

  if (!this->has_state() || this->hour_ != h || this->minute_ != m || this->second_ != s) {
    this->hour_ = h;
    this->minute_ = m;
    this->second_ = s;
    this->publish_state();
  }
}

void TubeClockDatetime::control(const datetime::TimeCall &call) {
  if (!this->parent_->is_initialized())
    return;

  uint8_t h = call.get_hour().value_or(this->hour_);
  uint8_t m = call.get_minute().value_or(this->minute_);
  uint8_t s = call.get_second().value_or(this->second_);

  this->parent_->set_alarm_slot(this->slot_, h, m, s);

  this->hour_ = h;
  this->minute_ = m;
  this->second_ = s;
  this->publish_state();
}

}  // namespace tube_clock
}  // namespace esphome

#endif  // USE_DATETIME_TIME
