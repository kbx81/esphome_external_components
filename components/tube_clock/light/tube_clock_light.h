/**
 * @file tube_clock_light.h
 * @brief Light platform for Tube Clock
 */

#pragma once

#include "esphome/core/component.h"
#include "esphome/components/light/light_output.h"
#include "../tube_clock.h"

namespace esphome::tube_clock {

class TubeClockLightOutput : public light::LightOutput, public TubeClockListener {
 public:
  explicit TubeClockLightOutput(TubeClock *parent) : parent_(parent) { parent->register_listener(this); }

  light::LightTraits get_traits() override {
    auto traits = light::LightTraits();
    traits.set_supported_color_modes({light::ColorMode::BRIGHTNESS});
    return traits;
  }

  void write_state(light::LightState *state) override {
    // Block all hardware commands until startup queries have completed
    if (!this->parent_->is_initialized())
      return;
    bool desired_on = state->current_values.is_on();
    // Only send HV command when the on/off state actually needs to change
    if (desired_on != this->parent_->get_hv_power()) {
      this->parent_->set_hv_power(desired_on);
    }
    if (!desired_on)
      return;
    float brightness = state->current_values.get_brightness();
    uint8_t intensity = static_cast<uint8_t>(roundf(brightness * 255.0f));
    // Skip if the clock already has this intensity: prevents echoing hardware-reported
    // values back to the clock (e.g. during auto-brightness updates)
    if (intensity != this->parent_->get_intensity()) {
      this->parent_->set_intensity(intensity);
    }
  }

  void on_tube_clock_update() override {
    if (this->state_ == nullptr)
      return;
    // On the very first hardware state notification (startup), always apply it even
    // if a transition is active. LightState::setup() may create a default transition
    // (e.g. 1s fade from ALWAYS_OFF restore mode) before startup queries finish; if
    // we skip the update the transition runs to {state=false}, and write_state() then
    // sends an unintended HVOF command once is_initialized() becomes true.
    // The instant-transition call below cancels any active transformer via set_immediately_().
    // After startup, ignore clock notifications during transitions so that intermediate
    // write_state() intensity commands (and their clock acknowledgements) do not interrupt
    // an in-progress user-initiated transition.
    if (this->startup_done_ && this->state_->is_transformer_active())
      return;
    bool hv_on = this->parent_->get_hv_power();
    uint8_t intensity = this->parent_->get_intensity();
    auto call = this->state_->make_call();
    if (!hv_on) {
      call.set_state(false);
    } else if (intensity > 0) {
      call.set_state(true);
      call.set_brightness(intensity / 255.0f);
    } else {
      // HV is on but intensity=0 (e.g. auto-brightness at minimum): skip the
      // update entirely. Feeding brightness=0.0f into LightCall triggers ESPHome's
      // auto-off which forces state=false, causing write_state() to send an
      // unintended power-off command to the clock. Calling set_state(true) without
      // a brightness would leave ESPHome using its stale remote_values brightness,
      // which write_state() would then echo back to the clock as intensity=255.
      return;
    }
    // Use instant transition so write_state() is called once with the hardware
    // value; the same-value check above prevents it from echoing back
    call.set_transition_length(0);
    call.perform();
    this->startup_done_ = true;
  }

  void setup_state(light::LightState *state) override { this->state_ = state; }

 protected:
  TubeClock *parent_;
  light::LightState *state_{nullptr};
  bool startup_done_{false};
};

class TubeClockRGBLightOutput : public light::LightOutput, public TubeClockListener {
 public:
  explicit TubeClockRGBLightOutput(TubeClock *parent) : parent_(parent) { parent->register_listener(this); }

  light::LightTraits get_traits() override {
    auto traits = light::LightTraits();
    traits.set_supported_color_modes({light::ColorMode::RGB});
    return traits;
  }

  void write_state(light::LightState *state) override {
    // Block all hardware commands until startup queries have completed
    if (!this->parent_->is_initialized())
      return;
    // The LED protocol carries intensity and color independently. Map ESPHome
    // brightness → LED master intensity and raw color components → LED RGB.
    bool desired_on = state->current_values.is_on();
    float brightness = desired_on ? state->current_values.get_brightness() : 0.0f;
    uint8_t intensity = static_cast<uint8_t>(roundf(brightness * 255.0f));
    uint8_t r = static_cast<uint8_t>(roundf(state->current_values.get_red() * 255.0f));
    uint8_t g = static_cast<uint8_t>(roundf(state->current_values.get_green() * 255.0f));
    uint8_t b = static_cast<uint8_t>(roundf(state->current_values.get_blue() * 255.0f));
    // Echo-suppression: compare against the normalized cache set by on_tube_clock_update.
    // ESPHome's ColorMode::RGB normalizes the color vector so the max component is always
    // 1.0 (255 in uint8). The cache stores values in this same normalized space, so the
    // comparison is consistent and hardware-originated syncs are never echoed back.
    // Color components are only compared when the LED is on (intensity > 0); when off
    // the color doesn't matter and we avoid echoing a spurious off-color command.
    if (intensity != this->cached_intensity_ ||
        (intensity > 0 && (r != this->cached_r_ || g != this->cached_g_ || b != this->cached_b_))) {
      // ESPHome applies gamma correction before write_state(), so the values here are
      // already gamma-corrected. Send gamma=false to skip the clock's internal gamma correction.
      this->parent_->set_led(intensity, r, g, b, false);
      this->cached_intensity_ = intensity;
      this->cached_r_ = r;
      this->cached_g_ = g;
      this->cached_b_ = b;
    }
  }

  void on_tube_clock_update() override {
    if (this->state_ == nullptr)
      return;
    // On the very first hardware state notification (startup), always apply it even
    // if a transition is active (see TubeClockLightOutput::on_tube_clock_update for
    // detailed explanation). After startup, respect active transitions.
    if (this->startup_done_ && this->state_->is_transformer_active())
      return;
    uint8_t intensity = this->parent_->get_led_intensity();
    uint8_t raw_r = this->parent_->get_led_r();
    uint8_t raw_g = this->parent_->get_led_g();
    uint8_t raw_b = this->parent_->get_led_b();
    // Normalize the RGB vector so max component = 1.0 before passing to ESPHome.
    // ColorMode::RGB applies the same normalization internally, so passing pre-normalized
    // values avoids any double-normalization and makes the cache calculation exact.
    uint8_t raw_max = std::max({raw_r, raw_g, raw_b});
    float r_f = raw_max > 0 ? raw_r / static_cast<float>(raw_max) : 0.0f;
    float g_f = raw_max > 0 ? raw_g / static_cast<float>(raw_max) : 0.0f;
    float b_f = raw_max > 0 ? raw_b / static_cast<float>(raw_max) : 0.0f;
    // Update the echo-suppression cache before call.perform(). write_state is invoked
    // deferred (from LightState::loop), so notifying_ flags don't reliably suppress it.
    // Storing the normalized values now ensures write_state sees a matching cache and
    // skips the hardware command regardless of when it fires.
    this->cached_intensity_ = intensity;
    this->cached_r_ = static_cast<uint8_t>(roundf(r_f * 255.0f));
    this->cached_g_ = static_cast<uint8_t>(roundf(g_f * 255.0f));
    this->cached_b_ = static_cast<uint8_t>(roundf(b_f * 255.0f));
    auto call = this->state_->make_call();
    if (intensity == 0) {
      call.set_state(false);
    } else {
      call.set_state(true);
      call.set_brightness(intensity / 255.0f);
      call.set_rgb(r_f, g_f, b_f);
    }
    call.set_transition_length(0);
    call.perform();
    this->startup_done_ = true;
  }

  void setup_state(light::LightState *state) override { this->state_ = state; }

 protected:
  TubeClock *parent_;
  light::LightState *state_{nullptr};
  bool startup_done_{false};
  uint8_t cached_intensity_{0};
  uint8_t cached_r_{0};
  uint8_t cached_g_{0};
  uint8_t cached_b_{0};
};

}  // namespace esphome::tube_clock
