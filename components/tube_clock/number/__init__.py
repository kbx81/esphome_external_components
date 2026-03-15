import esphome.codegen as cg
from esphome.components import number
import esphome.config_validation as cv
from esphome.const import (
    CONF_TYPE,
    DEVICE_CLASS_TEMPERATURE_DELTA,
    ENTITY_CATEGORY_CONFIG,
    UNIT_CELSIUS,
    UNIT_MILLISECOND,
    UNIT_SECOND,
)

from .. import TubeClock, tube_clock_ns

DEPENDENCIES = ["tube_clock"]

TubeClockNumber = tube_clock_ns.class_("TubeClockNumber", number.Number, cg.Component)

CONF_TUBE_CLOCK_ID = "tube_clock_id"

# Number types
TYPE_TIME_DISPLAY_DURATION = "time_display_duration"
TYPE_DATE_DISPLAY_DURATION = "date_display_duration"
TYPE_TEMP_DISPLAY_DURATION = "temp_display_duration"
TYPE_FADE_DURATION = "fade_duration"
TYPE_EFFECT_DURATION = "effect_duration"
TYPE_EFFECT_FREQUENCY = "effect_frequency"
TYPE_MINIMUM_INTENSITY = "minimum_intensity"
TYPE_BEEPER_VOLUME = "beeper_volume"
TYPE_TEMP_CAL_STM32 = "temp_cal_stm32"
TYPE_TEMP_CAL_DS3234 = "temp_cal_ds3234"
TYPE_TEMP_CAL_DS1722 = "temp_cal_ds1722"
TYPE_TEMP_CAL_LM74 = "temp_cal_lm74"
TYPE_TIMER_RESET_VALUE = "timer_reset_value"
TYPE_IDLE_TIMEOUT = "idle_timeout"

NUMBER_TYPES = {
    TYPE_TIME_DISPLAY_DURATION: "TimeDisplayDuration",
    TYPE_DATE_DISPLAY_DURATION: "DateDisplayDuration",
    TYPE_TEMP_DISPLAY_DURATION: "TempDisplayDuration",
    TYPE_FADE_DURATION: "FadeDuration",
    TYPE_EFFECT_DURATION: "EffectDuration",
    TYPE_EFFECT_FREQUENCY: "EffectFrequency",
    TYPE_MINIMUM_INTENSITY: "MinimumIntensity",
    TYPE_BEEPER_VOLUME: "BeeperVolume",
    TYPE_TEMP_CAL_STM32: "TempCalStm32",
    TYPE_TEMP_CAL_DS3234: "TempCalDs3234",
    TYPE_TEMP_CAL_DS1722: "TempCalDs1722",
    TYPE_TEMP_CAL_LM74: "TempCalLm74",
    TYPE_TIMER_RESET_VALUE: "TimerResetValue",
    TYPE_IDLE_TIMEOUT: "IdleTimeout",
}

# (min_value, max_value, step) for each number type
NUMBER_TYPE_RANGES = {
    TYPE_TIME_DISPLAY_DURATION: (1, 300, 1),
    TYPE_DATE_DISPLAY_DURATION: (1, 300, 1),
    TYPE_TEMP_DISPLAY_DURATION: (1, 300, 1),
    TYPE_FADE_DURATION: (1, 1000, 1),
    TYPE_EFFECT_DURATION: (1, 1000, 1),
    TYPE_EFFECT_FREQUENCY: (1, 43200, 1),
    TYPE_MINIMUM_INTENSITY: (1, 1000, 1),
    TYPE_BEEPER_VOLUME: (0, 7, 1),
    TYPE_TEMP_CAL_STM32: (-9.9, 9.9, 0.1),
    TYPE_TEMP_CAL_DS3234: (-9.9, 9.9, 0.1),
    TYPE_TEMP_CAL_DS1722: (-9.9, 9.9, 0.1),
    TYPE_TEMP_CAL_LM74: (-9.9, 9.9, 0.1),
    TYPE_TIMER_RESET_VALUE: (0, 65535, 1),
    TYPE_IDLE_TIMEOUT: (10, 600, 1),
}

def _number_schema(base_schema):
    return base_schema.extend({cv.GenerateID(CONF_TUBE_CLOCK_ID): cv.use_id(TubeClock)})


CONFIG_SCHEMA = cv.typed_schema(
    {
        TYPE_TIME_DISPLAY_DURATION: _number_schema(
            number.number_schema(
                TubeClockNumber,
                entity_category=ENTITY_CATEGORY_CONFIG,
                unit_of_measurement=UNIT_SECOND,
                icon="mdi:timer",
            )
        ),
        TYPE_DATE_DISPLAY_DURATION: _number_schema(
            number.number_schema(
                TubeClockNumber,
                entity_category=ENTITY_CATEGORY_CONFIG,
                unit_of_measurement=UNIT_SECOND,
                icon="mdi:timer",
            )
        ),
        TYPE_TEMP_DISPLAY_DURATION: _number_schema(
            number.number_schema(
                TubeClockNumber,
                entity_category=ENTITY_CATEGORY_CONFIG,
                unit_of_measurement=UNIT_SECOND,
                icon="mdi:timer",
            )
        ),
        TYPE_FADE_DURATION: _number_schema(
            number.number_schema(
                TubeClockNumber,
                entity_category=ENTITY_CATEGORY_CONFIG,
                unit_of_measurement=UNIT_MILLISECOND,
                icon="mdi:timer",
            )
        ),
        TYPE_EFFECT_DURATION: _number_schema(
            number.number_schema(
                TubeClockNumber,
                entity_category=ENTITY_CATEGORY_CONFIG,
                unit_of_measurement=UNIT_MILLISECOND,
                icon="mdi:timer",
            )
        ),
        TYPE_EFFECT_FREQUENCY: _number_schema(
            number.number_schema(
                TubeClockNumber,
                entity_category=ENTITY_CATEGORY_CONFIG,
                unit_of_measurement=UNIT_SECOND,
                icon="mdi:timer",
            )
        ),
        TYPE_MINIMUM_INTENSITY: _number_schema(
            number.number_schema(
                TubeClockNumber,
                entity_category=ENTITY_CATEGORY_CONFIG,
                icon="mdi:brightness-6",
            )
        ),
        TYPE_BEEPER_VOLUME: _number_schema(
            number.number_schema(
                TubeClockNumber,
                entity_category=ENTITY_CATEGORY_CONFIG,
                icon="mdi:volume-high",
            )
        ),
        TYPE_TEMP_CAL_STM32: _number_schema(
            number.number_schema(
                TubeClockNumber,
                entity_category=ENTITY_CATEGORY_CONFIG,
                device_class=DEVICE_CLASS_TEMPERATURE_DELTA,
                unit_of_measurement=UNIT_CELSIUS,
                icon="mdi:thermometer",
            )
        ),
        TYPE_TEMP_CAL_DS3234: _number_schema(
            number.number_schema(
                TubeClockNumber,
                entity_category=ENTITY_CATEGORY_CONFIG,
                device_class=DEVICE_CLASS_TEMPERATURE_DELTA,
                unit_of_measurement=UNIT_CELSIUS,
                icon="mdi:thermometer",
            )
        ),
        TYPE_TEMP_CAL_DS1722: _number_schema(
            number.number_schema(
                TubeClockNumber,
                entity_category=ENTITY_CATEGORY_CONFIG,
                device_class=DEVICE_CLASS_TEMPERATURE_DELTA,
                unit_of_measurement=UNIT_CELSIUS,
                icon="mdi:thermometer",
            )
        ),
        TYPE_TEMP_CAL_LM74: _number_schema(
            number.number_schema(
                TubeClockNumber,
                entity_category=ENTITY_CATEGORY_CONFIG,
                device_class=DEVICE_CLASS_TEMPERATURE_DELTA,
                unit_of_measurement=UNIT_CELSIUS,
                icon="mdi:thermometer",
            )
        ),
        TYPE_TIMER_RESET_VALUE: _number_schema(
            number.number_schema(
                TubeClockNumber,
                entity_category=ENTITY_CATEGORY_CONFIG,
                unit_of_measurement=UNIT_SECOND,
                icon="mdi:timer-settings",
            )
        ),
        TYPE_IDLE_TIMEOUT: _number_schema(
            number.number_schema(
                TubeClockNumber,
                entity_category=ENTITY_CATEGORY_CONFIG,
                unit_of_measurement=UNIT_SECOND,
                icon="mdi:timer-sand-complete",
            )
        ),
    },
    key=CONF_TYPE,
)


async def to_code(config):
    """Generate code for tube_clock number."""
    min_value, max_value, step = NUMBER_TYPE_RANGES[config[CONF_TYPE]]
    var = await number.new_number(config, min_value=min_value, max_value=max_value, step=step)
    await cg.register_component(var, config)

    parent = await cg.get_variable(config[CONF_TUBE_CLOCK_ID])
    cg.add(var.set_parent(parent))
    cg.add(var.set_type(NUMBER_TYPES[config[CONF_TYPE]]))
