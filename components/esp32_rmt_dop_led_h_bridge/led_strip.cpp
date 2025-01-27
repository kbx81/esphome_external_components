#include "led_strip.h"
#include <cinttypes>

#ifdef USE_ESP32

#include "esphome/core/helpers.h"
#include "esphome/core/log.h"

#include <esp_attr.h>

namespace esphome {
namespace esp32_rmt_dop_led_h_bridge {

static const char *const TAG = "esp32_rmt_dop_led_h_bridge";

#ifdef USE_ESP32_VARIANT_ESP32H2
static const uint32_t RMT_CLK_FREQ = 32000000;
static const uint8_t RMT_CLK_DIV = 1;
#else
static const uint32_t RMT_CLK_FREQ = 80000000;
static const uint8_t RMT_CLK_DIV = 2;
#endif

void ESP32RMTDoPLEDHBridgeLightOutput::setup() {
  ESP_LOGCONFIG(TAG, "Setting up ESP32 RMT DoP LED with H-bridge...");

  size_t buffer_size = this->get_buffer_size_();

  RAMAllocator<uint8_t> allocator(this->use_psram_ ? 0 : RAMAllocator<uint8_t>::ALLOC_INTERNAL);
  this->buf_ = allocator.allocate(buffer_size);
  if (this->buf_ == nullptr) {
    ESP_LOGE(TAG, "Cannot allocate LED buffer!");
    this->mark_failed();
    return;
  }
  memset(this->buf_, 0, buffer_size);

  this->effect_data_ = allocator.allocate(this->num_leds_);
  if (this->effect_data_ == nullptr) {
    ESP_LOGE(TAG, "Cannot allocate effect data!");
    this->mark_failed();
    return;
  }

#if ESP_IDF_VERSION_MAJOR >= 5
  RAMAllocator<rmt_symbol_word_t> rmt_allocator(this->use_psram_ ? 0 : RAMAllocator<rmt_symbol_word_t>::ALLOC_INTERNAL);

  // header bits + reset bit + 8 bits per byte, 1 rmt_symbol_word_t per bit
  this->rmt_buf_ = rmt_allocator.allocate(((this->num_header_bits_ + 1) * this->num_leds_) + buffer_size * 8 + 1);

  rmt_tx_channel_config_t channel;
  memset(&channel, 0, sizeof(channel));
  channel.clk_src = RMT_CLK_SRC_DEFAULT;
  channel.resolution_hz = RMT_CLK_FREQ / RMT_CLK_DIV;
  channel.gpio_num = gpio_num_t(this->pin_);
  channel.mem_block_symbols = this->rmt_symbols_;
  channel.trans_queue_depth = 1;
  channel.flags.io_loop_back = 0;
  channel.flags.io_od_mode = 1;
  channel.flags.invert_out = 0;
  channel.flags.with_dma = 0;
  channel.intr_priority = 0;
  if (rmt_new_tx_channel(&channel, &this->channel_) != ESP_OK) {
    ESP_LOGE(TAG, "Channel creation failed");
    this->mark_failed();
    return;
  }

  rmt_copy_encoder_config_t encoder;
  memset(&encoder, 0, sizeof(encoder));
  if (rmt_new_copy_encoder(&encoder, &this->encoder_) != ESP_OK) {
    ESP_LOGE(TAG, "Encoder creation failed");
    this->mark_failed();
    return;
  }

  if (rmt_enable(this->channel_) != ESP_OK) {
    ESP_LOGE(TAG, "Enabling channel failed");
    this->mark_failed();
    return;
  }
#else
  RAMAllocator<rmt_item32_t> rmt_allocator(this->use_psram_ ? 0 : RAMAllocator<rmt_item32_t>::ALLOC_INTERNAL);

  // header bits + 8 bits per byte, 1 rmt_item32_t per bit + 1 rmt_item32_t for reset
  this->rmt_buf_ = rmt_allocator.allocate(this->num_header_bits_ + buffer_size * 8 + 1);

  rmt_config_t config;
  memset(&config, 0, sizeof(config));
  config.channel = this->channel_;
  config.rmt_mode = RMT_MODE_TX;
  config.gpio_num = gpio_num_t(this->pin_);
  config.mem_block_num = 1;
  config.clk_div = RMT_CLK_DIV;
  config.tx_config.loop_en = false;
  config.tx_config.carrier_level = RMT_CARRIER_LEVEL_LOW;
  config.tx_config.carrier_en = false;
  config.tx_config.idle_level = RMT_IDLE_LEVEL_LOW;
  config.tx_config.idle_output_en = true;

  if (rmt_config(&config) != ESP_OK) {
    ESP_LOGE(TAG, "Cannot initialize RMT!");
    this->mark_failed();
    return;
  }
  if (rmt_driver_install(config.channel, 0, 0) != ESP_OK) {
    ESP_LOGE(TAG, "Cannot install RMT driver!");
    this->mark_failed();
    return;
  }
#endif
}

void ESP32RMTDoPLEDHBridgeLightOutput::set_led_params(uint32_t bit0_high, uint32_t bit0_low, uint32_t bit1_high,
                                                      uint32_t bit1_low, uint32_t reset_time_high,
                                                      uint32_t reset_time_low) {
  float ratio = (float) RMT_CLK_FREQ / RMT_CLK_DIV / 1e09f;

  // 0-bit
  this->bit0_.duration0 = (uint32_t) (ratio * bit0_high);
  this->bit0_.level0 = 1;
  this->bit0_.duration1 = (uint32_t) (ratio * bit0_low);
  this->bit0_.level1 = 0;
  // 1-bit
  this->bit1_.duration0 = (uint32_t) (ratio * bit1_high);
  this->bit1_.level0 = 1;
  this->bit1_.duration1 = (uint32_t) (ratio * bit1_low);
  this->bit1_.level1 = 0;
  // reset
  this->reset_.duration0 = (uint32_t) (ratio * reset_time_high);
  this->reset_.level0 = 1;
  this->reset_.duration1 = (uint32_t) (ratio * reset_time_low);
  this->reset_.level1 = 0;
}

void ESP32RMTDoPLEDHBridgeLightOutput::write_state(light::LightState *state) {
  auto color_mode = this->is_effect_active() ? light::ColorMode::RGB : state->current_values.get_color_mode();
  uint32_t now = micros();
  float brightness;
  state->current_values_as_brightness(&brightness);
  if (state->current_values.is_on()) {
    switch (color_mode) {
      case light::ColorMode::RGB: {
        if (color_mode != this->prev_color_mode_) {
          this->prev_color_mode_ = color_mode;  // save new mode
          this->output_n1_pwm_->set_level(0);   // turn off conflicting drivers
          this->output_p2_->turn_off();         // turn off conflicting drivers
          delay(2);                             // let currents settle
          this->output_n2_->turn_on();          // turn on required drivers
          this->output_2v5_->turn_on();         // turn on required drivers
        }

        // protect from refreshing too often
        if (this->max_refresh_rate_.has_value()) {
          if (this->max_refresh_rate_.value() != 0 && (now - this->last_refresh_) < this->max_refresh_rate_.value()) {
            // try again next loop iteration, so that this change won't get lost
            this->schedule_show();
            return;
          }
        }

        ESP_LOGVV(TAG, "Writing RGB values to bus...");

#if ESP_IDF_VERSION_MAJOR >= 5
        esp_err_t error = rmt_tx_wait_all_done(this->channel_, 1000);
#else
        esp_err_t error = rmt_wait_tx_done(this->channel_, pdMS_TO_TICKS(1000));
#endif
        if (error != ESP_OK) {
          ESP_LOGE(TAG, "RMT TX timeout");
          this->status_set_warning();
          return;
        }
        delayMicroseconds(50);

        size_t buffer_size = this->get_buffer_size_();

        size_t size = 0;
        size_t len = 0;
        size_t led = 0;
        uint8_t *psrc = this->buf_;
        uint8_t divisor = this->is_rgbw_ ? 4 : 3;
#if ESP_IDF_VERSION_MAJOR >= 5
        rmt_symbol_word_t *pdest = this->rmt_buf_;
#else
        rmt_item32_t *pdest = this->rmt_buf_;
#endif
        while (size < buffer_size) {
          if (!(size % divisor)) {
            if (size) {  // insert "reset" if not the first LED
              if (this->reset_.duration0 > 0 || this->reset_.duration1 > 0) {
                pdest->val = this->reset_.val;
                pdest++;
                len++;
              }
            }
            // insert header
            for (int i = 0; i < this->num_header_bits_; i++) {
              pdest->val = led & (1 << i) ? this->bit1_.val : this->bit0_.val;
              pdest++;
              len++;
            }
            led++;
          }

          uint8_t b = *psrc;
          for (int i = 0; i < 8; i++) {
            pdest->val = b & (1 << i) ? this->bit1_.val : this->bit0_.val;
            pdest++;
            len++;
          }
          size++;
          psrc++;
        }

        if (this->reset_.duration0 > 0 || this->reset_.duration1 > 0) {
          pdest->val = this->reset_.val;
          pdest++;
          len++;
        }

#if ESP_IDF_VERSION_MAJOR >= 5
        rmt_transmit_config_t config;
        memset(&config, 0, sizeof(config));
        config.loop_count = 0;
        config.flags.eot_level = 0;
        error = rmt_transmit(this->channel_, this->encoder_, this->rmt_buf_, len * sizeof(rmt_symbol_word_t), &config);
#else
        error = rmt_write_items(this->channel_, this->rmt_buf_, len, false);
#endif
        if (error != ESP_OK) {
          ESP_LOGE(TAG, "RMT TX error");
          this->status_set_warning();
          return;
        }
        this->status_clear_warning();
        break;
      }

      case light::ColorMode::WHITE:
        if (color_mode != this->prev_color_mode_) {
          this->prev_color_mode_ = color_mode;  // save new mode
          this->output_2v5_->turn_off();        // turn off conflicting drivers
          this->output_n2_->turn_off();         // turn off conflicting drivers
          this->set_rmt_output_high();          // turn off conflicting drivers
          delay(2);                             // let currents settle
          this->output_p2_->turn_on();          // turn on required drivers
        }
        this->output_n1_pwm_->set_level(brightness);  // set brightness
        break;

      default:
        break;
    }
    this->last_refresh_ = now;
    this->mark_shown_();
  } else {  // turn everything off
    this->prev_color_mode_ = light::ColorMode::UNKNOWN;
    this->output_2v5_->turn_off();
    this->output_n2_->turn_off();
    this->set_rmt_output_high();
    this->output_n1_pwm_->set_level(0);
    this->output_p2_->turn_off();
  }
}

void ESP32RMTDoPLEDHBridgeLightOutput::set_rmt_output_high() {
  if (this->channel_ != nullptr) {
#if ESP_IDF_VERSION_MAJOR >= 5
    rmt_transmit_config_t config;
    memset(&config, 0, sizeof(config));
    config.loop_count = 0;
    config.flags.eot_level = 1;
    this->rmt_buf_->val = 0;
    this->rmt_buf_->level0 = 1;
    this->rmt_buf_->level1 = 1;
    esp_err_t error = rmt_transmit(this->channel_, this->encoder_, this->rmt_buf_, sizeof(rmt_symbol_word_t), &config);
#else
    esp_err_t error = rmt_write_items(this->channel_, this->rmt_buf_, 1, false);
#endif
    if (error != ESP_OK) {
      ESP_LOGE(TAG, "RMT TX error");
      this->status_set_warning();
      return;
    }
    this->status_clear_warning();
  }
}

light::ESPColorView ESP32RMTDoPLEDHBridgeLightOutput::get_view_internal(int32_t index) const {
  int32_t r = 0, g = 0, b = 0;
  switch (this->rgb_order_) {
    case ORDER_RGB:
      r = 0;
      g = 1;
      b = 2;
      break;
    case ORDER_RBG:
      r = 0;
      g = 2;
      b = 1;
      break;
    case ORDER_GRB:
      r = 1;
      g = 0;
      b = 2;
      break;
    case ORDER_GBR:
      r = 2;
      g = 0;
      b = 1;
      break;
    case ORDER_BGR:
      r = 2;
      g = 1;
      b = 0;
      break;
    case ORDER_BRG:
      r = 1;
      g = 2;
      b = 0;
      break;
  }
  uint8_t multiplier = this->is_rgbw_ || this->is_wrgb_ ? 4 : 3;
  uint8_t white = this->is_wrgb_ ? 0 : 3;

  return {this->buf_ + (index * multiplier) + r + this->is_wrgb_,
          this->buf_ + (index * multiplier) + g + this->is_wrgb_,
          this->buf_ + (index * multiplier) + b + this->is_wrgb_,
          this->is_rgbw_ || this->is_wrgb_ ? this->buf_ + (index * multiplier) + white : nullptr,
          &this->effect_data_[index],
          &this->correction_};
}

void ESP32RMTDoPLEDHBridgeLightOutput::dump_config() {
  ESP_LOGCONFIG(TAG, "ESP32 RMT DoP LED with H-bridge:");
  ESP_LOGCONFIG(TAG, "  Pin: %u", this->pin_);
#if ESP_IDF_VERSION_MAJOR >= 5
  ESP_LOGCONFIG(TAG, "  RMT Symbols: %" PRIu32, this->rmt_symbols_);
#else
  ESP_LOGCONFIG(TAG, "  Channel: %u", this->channel_);
#endif
  const char *rgb_order;
  switch (this->rgb_order_) {
    case ORDER_RGB:
      rgb_order = "RGB";
      break;
    case ORDER_RBG:
      rgb_order = "RBG";
      break;
    case ORDER_GRB:
      rgb_order = "GRB";
      break;
    case ORDER_GBR:
      rgb_order = "GBR";
      break;
    case ORDER_BGR:
      rgb_order = "BGR";
      break;
    case ORDER_BRG:
      rgb_order = "BRG";
      break;
    default:
      rgb_order = "UNKNOWN";
      break;
  }
  ESP_LOGCONFIG(TAG, "  RGB Order: %s", rgb_order);
  ESP_LOGCONFIG(TAG, "  H-bridge enabled: %s", YESNO(this->enable_h_bridge_));
  ESP_LOGCONFIG(TAG, "  Max refresh rate: %" PRIu32, *this->max_refresh_rate_);
  ESP_LOGCONFIG(TAG, "  Number of header bits: %u", this->num_header_bits_);
  ESP_LOGCONFIG(TAG, "  Number of LEDs: %u", this->num_leds_);
}

float ESP32RMTDoPLEDHBridgeLightOutput::get_setup_priority() const { return setup_priority::HARDWARE; }

std::unique_ptr<light::LightTransformer> ESP32RMTDoPLEDHBridgeLightOutput::create_default_transition() {
  return make_unique<ESP32RMTDoPLEDHBridgeLightTransformer>(*this);
}

void ESP32RMTDoPLEDHBridgeLightTransformer::start() {
  // don't try to transition over running effects.
  if (this->light_.is_effect_active())
    return;

  // When turning light on from off state, use target state and only increase brightness from zero.
  if (!this->start_values_.is_on() && this->target_values_.is_on()) {
    this->start_values_ = light::LightColorValues(this->target_values_);
    this->start_values_.set_brightness(0.0f);
  }

  // When turning light off from on state, use source state and only decrease brightness to zero. Use a second
  // variable for transition end state, as overwriting target_values breaks LightState logic.
  if (this->start_values_.is_on() && !this->target_values_.is_on()) {
    this->end_values_ = light::LightColorValues(this->start_values_);
    this->end_values_.set_brightness(0.0f);
  } else {
    this->end_values_ = light::LightColorValues(this->target_values_);
  }

  // When changing color mode, go through off state, as color modes are orthogonal and there can't be two active.
  if (this->start_values_.get_color_mode() != this->target_values_.get_color_mode()) {
    this->changing_color_mode_ = true;
    this->intermediate_values_ = this->start_values_;
    this->intermediate_values_.set_state(false);
    this->intermediate_values_.set_brightness(0);
  }

  auto end_values = this->target_values_;
  this->target_color_ = color_from_light_color_values(end_values);

  // our transition will handle brightness, disable brightness in correction.
  this->light_.correction_.set_local_brightness(255);
  this->target_color_ *= light::to_uint8_scale(end_values.get_brightness() * end_values.get_state());
}

optional<light::LightColorValues> ESP32RMTDoPLEDHBridgeLightTransformer::apply() {
  float p = this->get_progress_();
  float smoothed_progress = light::LightTransitionTransformer::smoothed_progress(p);

  // When running an output-buffer modifying effect, don't try to transition individual LEDs, but instead just fade the
  // LightColorValues. write_state() then picks up the change in brightness, and the color change is picked up by the
  // effects which respect it.
  if (this->light_.is_effect_active())
    return light::LightColorValues::lerp(this->get_start_values(), this->get_target_values(), smoothed_progress);

  // Use a specialized transition for addressable lights: instead of using a unified transition for
  // all LEDs, we use the current state of each LED as the start.

  // We can't use a direct lerp smoothing here though - that would require creating a copy of the original
  // state of each LED at the start of the transition.
  // Instead, we "fake" the look of the LERP by using an exponential average over time and using
  // dynamically-calculated alpha values to match the look.

  float denom = (1.0f - smoothed_progress);
  float alpha = denom == 0.0f ? 1.0f : (smoothed_progress - this->last_transition_progress_) / denom;

  // We need to use a low-resolution alpha here which makes the transition set in only after ~half of the length
  // We solve this by accumulating the fractional part of the alpha over time.
  float alpha255 = alpha * 255.0f;
  float alpha255int = floorf(alpha255);
  float alpha255remainder = alpha255 - alpha255int;

  this->accumulated_alpha_ += alpha255remainder;
  float alpha_add = floorf(this->accumulated_alpha_);
  this->accumulated_alpha_ -= alpha_add;

  alpha255 += alpha_add;
  alpha255 = clamp(alpha255, 0.0f, 255.0f);
  auto alpha8 = static_cast<uint8_t>(alpha255);

  if (alpha8 != 0) {
    uint8_t inv_alpha8 = 255 - alpha8;
    Color add = this->target_color_ * alpha8;

    for (auto led : this->light_)
      led.set(add + led.get() * inv_alpha8);
  }

  this->last_transition_progress_ = smoothed_progress;
  this->light_.schedule_show();

  // Halfway through, when intermediate state (off) is reached, flip it to the target, but remain off.
  if (this->changing_color_mode_ && p > 0.5f &&
      this->intermediate_values_.get_color_mode() != this->target_values_.get_color_mode()) {
    this->intermediate_values_ = this->target_values_;
    this->intermediate_values_.set_state(false);
    this->intermediate_values_.set_brightness(0);
  }

  light::LightColorValues &start =
      this->changing_color_mode_ && p > 0.5f ? this->intermediate_values_ : this->start_values_;
  light::LightColorValues &end =
      this->changing_color_mode_ && p < 0.5f ? this->intermediate_values_ : this->end_values_;
  if (this->changing_color_mode_)
    p = p < 0.5f ? p * 2 : (p - 0.5) * 2;

  float v = light::LightTransitionTransformer::smoothed_progress(p);
  return light::LightColorValues::lerp(start, end, v);
}

}  // namespace esp32_rmt_dop_led_h_bridge
}  // namespace esphome

#endif  // USE_ESP32
