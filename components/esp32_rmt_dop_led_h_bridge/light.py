from dataclasses import dataclass
import logging

from esphome import pins
import esphome.codegen as cg
from esphome.components import esp32_rmt, light, output
import esphome.config_validation as cv
from esphome.const import (
    CONF_CHIPSET,
    CONF_IS_RGBW,
    CONF_MAX_REFRESH_RATE,
    CONF_NUM_LEDS,
    CONF_OPEN_DRAIN,
    CONF_OUTPUT_ID,
    CONF_PIN,
    CONF_RGB_ORDER,
    CONF_RMT_CHANNEL,
    CONF_RMT_SYMBOLS,
)
from esphome.core import CORE

_LOGGER = logging.getLogger(__name__)

CODEOWNERS = ["@kbx81"]
DEPENDENCIES = ["esp32"]

esp32_rmt_dop_led_h_bridge_ns = cg.esphome_ns.namespace("esp32_rmt_dop_led_h_bridge")
ESP32RMTDoPLEDHBridgeLightOutput = esp32_rmt_dop_led_h_bridge_ns.class_(
    "ESP32RMTDoPLEDHBridgeLightOutput", light.AddressableLight
)

RGBOrder = esp32_rmt_dop_led_h_bridge_ns.enum("RGBOrder")

RGB_ORDERS = {
    "RGB": RGBOrder.ORDER_RGB,
    "RBG": RGBOrder.ORDER_RBG,
    "GRB": RGBOrder.ORDER_GRB,
    "GBR": RGBOrder.ORDER_GBR,
    "BGR": RGBOrder.ORDER_BGR,
    "BRG": RGBOrder.ORDER_BRG,
}


@dataclass
class LEDStripTimings:
    bit0_high: int
    bit0_low: int
    bit1_high: int
    bit1_low: int
    reset_high: int
    reset_low: int


CHIPSETS = {
    "WS2811": LEDStripTimings(300, 1090, 1090, 320, 0, 300000),
    "WS2812": LEDStripTimings(400, 1000, 1000, 400, 0, 0),
    "SK6812": LEDStripTimings(300, 900, 600, 600, 0, 0),
    "APA106": LEDStripTimings(350, 1360, 1360, 350, 0, 0),
    "SM16703": LEDStripTimings(300, 900, 900, 300, 0, 0),
    "HONEYWELL_5V_DOP": LEDStripTimings(80000, 80000, 140000, 80000, 260000, 260000),
}

CONF_BIT0_HIGH = "bit0_high"
CONF_BIT0_LOW = "bit0_low"
CONF_BIT1_HIGH = "bit1_high"
CONF_BIT1_LOW = "bit1_low"
CONF_ENABLE_H_BRIDGE = "enable_h_bridge"
CONF_IS_WRGB = "is_wrgb"
CONF_LSB_FIRST = "lsb_first"
CONF_NUM_HEADER_BITS = "num_header_bits"
CONF_OUTPUT_2V5_ID = "output_2v5_id"
CONF_OUTPUT_N1_PWM_ID = "output_n1_pwm_id"
CONF_OUTPUT_N2_ID = "output_n2_id"
CONF_OUTPUT_P2_ID = "output_p2_id"
CONF_RESET_HIGH = "reset_high"
CONF_RESET_LOW = "reset_low"
CONF_USE_PSRAM = "use_psram"


class OptionalForIDF5(cv.SplitDefault):
    @property
    def default(self):
        if not esp32_rmt.use_new_rmt_driver():
            return cv.UNDEFINED
        return super().default

    @default.setter
    def default(self, value):
        # Ignore default set from vol.Optional
        pass


def only_with_new_rmt_driver(obj):
    if not esp32_rmt.use_new_rmt_driver():
        raise cv.Invalid(
            "This feature is only available for the IDF framework version 5."
        )
    return obj


def not_with_new_rmt_driver(obj):
    if esp32_rmt.use_new_rmt_driver():
        raise cv.Invalid(
            "This feature is not available for the IDF framework version 5."
        )
    return obj


def final_validation(config):
    if not esp32_rmt.use_new_rmt_driver():
        if CONF_RMT_CHANNEL not in config:
            if CORE.using_esp_idf:
                raise cv.Invalid(
                    "rmt_channel is a required option for IDF version < 5."
                )
            raise cv.Invalid(
                "rmt_channel is a required option for the Arduino framework."
            )
        _LOGGER.warning(
            "RMT_LED_STRIP support for IDF version < 5 is deprecated and will be removed soon."
        )


FINAL_VALIDATE_SCHEMA = final_validation


CONFIG_SCHEMA = cv.All(
    light.ADDRESSABLE_LIGHT_SCHEMA.extend(
        {
            cv.GenerateID(CONF_OUTPUT_ID): cv.declare_id(
                ESP32RMTDoPLEDHBridgeLightOutput
            ),
            cv.Required(CONF_PIN): pins.internal_gpio_output_pin_number,
            cv.Optional(CONF_LSB_FIRST, default=False): cv.boolean,
            cv.Required(CONF_NUM_HEADER_BITS): cv.positive_not_null_int,
            cv.Optional(CONF_OPEN_DRAIN, default=False): cv.boolean,
            cv.Required(CONF_OUTPUT_2V5_ID): cv.use_id(output.BinaryOutput),
            cv.Required(CONF_OUTPUT_P2_ID): cv.use_id(output.BinaryOutput),
            cv.Required(CONF_OUTPUT_N1_PWM_ID): cv.use_id(output.FloatOutput),
            cv.Required(CONF_OUTPUT_N2_ID): cv.use_id(output.BinaryOutput),
            cv.Required(CONF_NUM_LEDS): cv.positive_not_null_int,
            cv.Required(CONF_RGB_ORDER): cv.enum(RGB_ORDERS, upper=True),
            cv.Optional(CONF_RMT_CHANNEL): cv.All(
                not_with_new_rmt_driver, esp32_rmt.validate_rmt_channel(tx=True)
            ),
            OptionalForIDF5(
                CONF_RMT_SYMBOLS,
                esp32_idf=192,
                esp32_s2_idf=192,
                esp32_s3_idf=192,
                esp32_c3_idf=96,
                esp32_c6_idf=96,
                esp32_h2_idf=96,
            ): cv.All(only_with_new_rmt_driver, cv.int_range(min=2)),
            cv.Optional(CONF_MAX_REFRESH_RATE): cv.positive_time_period_microseconds,
            cv.Optional(CONF_CHIPSET): cv.one_of(*CHIPSETS, upper=True),
            cv.Optional(CONF_ENABLE_H_BRIDGE, default=False): cv.boolean,
            cv.Optional(CONF_IS_RGBW, default=False): cv.boolean,
            cv.Optional(CONF_IS_WRGB, default=False): cv.boolean,
            cv.Optional(CONF_USE_PSRAM, default=True): cv.boolean,
            cv.Inclusive(
                CONF_BIT0_HIGH,
                "custom",
            ): cv.positive_time_period_nanoseconds,
            cv.Inclusive(
                CONF_BIT0_LOW,
                "custom",
            ): cv.positive_time_period_nanoseconds,
            cv.Inclusive(
                CONF_BIT1_HIGH,
                "custom",
            ): cv.positive_time_period_nanoseconds,
            cv.Inclusive(
                CONF_BIT1_LOW,
                "custom",
            ): cv.positive_time_period_nanoseconds,
            cv.Optional(
                CONF_RESET_HIGH,
                default="0 us",
            ): cv.positive_time_period_nanoseconds,
            cv.Optional(
                CONF_RESET_LOW,
                default="0 us",
            ): cv.positive_time_period_nanoseconds,
        }
    ).extend(cv.COMPONENT_SCHEMA),
    cv.has_exactly_one_key(CONF_CHIPSET, CONF_BIT0_HIGH),
)


async def to_code(config):
    output_2v5 = await cg.get_variable(config[CONF_OUTPUT_2V5_ID])
    output_n1_pwm = await cg.get_variable(config[CONF_OUTPUT_N1_PWM_ID])
    output_n2 = await cg.get_variable(config[CONF_OUTPUT_N2_ID])
    output_p2 = await cg.get_variable(config[CONF_OUTPUT_P2_ID])
    var = cg.new_Pvariable(config[CONF_OUTPUT_ID])
    await light.register_light(var, config)
    await cg.register_component(var, config)

    cg.add(var.set_lsb_first(config[CONF_LSB_FIRST]))
    cg.add(var.set_num_header_bits(config[CONF_NUM_HEADER_BITS]))
    cg.add(var.set_num_leds(config[CONF_NUM_LEDS]))
    cg.add(var.set_open_drain(config[CONF_OPEN_DRAIN]))
    cg.add(var.set_output_2v5(output_2v5))
    cg.add(var.set_output_n1_pwm(output_n1_pwm))
    cg.add(var.set_output_n2(output_n2))
    cg.add(var.set_output_p2(output_p2))
    cg.add(var.set_pin(config[CONF_PIN]))

    if CONF_MAX_REFRESH_RATE in config:
        cg.add(var.set_max_refresh_rate(config[CONF_MAX_REFRESH_RATE]))

    if CONF_CHIPSET in config:
        chipset = CHIPSETS[config[CONF_CHIPSET]]
        cg.add(
            var.set_led_params(
                chipset.bit0_high,
                chipset.bit0_low,
                chipset.bit1_high,
                chipset.bit1_low,
                chipset.reset_high,
                chipset.reset_low,
            )
        )
    else:
        cg.add(
            var.set_led_params(
                config[CONF_BIT0_HIGH],
                config[CONF_BIT0_LOW],
                config[CONF_BIT1_HIGH],
                config[CONF_BIT1_LOW],
                config[CONF_RESET_HIGH],
                config[CONF_RESET_LOW],
            )
        )

    cg.add(var.set_rgb_order(config[CONF_RGB_ORDER]))
    cg.add(var.set_enable_h_bridge(config[CONF_ENABLE_H_BRIDGE]))
    cg.add(var.set_is_rgbw(config[CONF_IS_RGBW]))
    cg.add(var.set_is_wrgb(config[CONF_IS_WRGB]))
    cg.add(var.set_use_psram(config[CONF_USE_PSRAM]))

    if esp32_rmt.use_new_rmt_driver():
        cg.add(var.set_rmt_symbols(config[CONF_RMT_SYMBOLS]))
    else:
        rmt_channel_t = cg.global_ns.enum("rmt_channel_t")
        cg.add(
            var.set_rmt_channel(
                getattr(rmt_channel_t, f"RMT_CHANNEL_{config[CONF_RMT_CHANNEL]}")
            )
        )
