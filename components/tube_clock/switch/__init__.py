import esphome.codegen as cg
from esphome.components import switch
import esphome.config_validation as cv
from esphome.const import CONF_TYPE, ENTITY_CATEGORY_CONFIG

CONF_SLOT = "slot"

from .. import TubeClock, tube_clock_ns

DEPENDENCIES = ["tube_clock"]

TubeClockSwitch = tube_clock_ns.class_("TubeClockSwitch", switch.Switch, cg.Component)

CONF_TUBE_CLOCK_ID = "tube_clock_id"

# Switch types
TYPE_HV_POWER = "hv_power"
TYPE_AUTO_BRIGHTNESS_NIXIE = "auto_brightness_nixie"
TYPE_AUTO_BRIGHTNESS_RGB = "auto_brightness_rgb"
TYPE_HOURLY_CHIME = "hourly_chime"
TYPE_SHOW_LEADING_ZEROS = "show_leading_zeros"
TYPE_STATUS_LED_AS_AM_PM = "status_led_as_am_pm"
TYPE_ALARM_BEEP = "alarm_beep"
TYPE_ALARM_BLINK = "alarm_blink"

SWITCH_TYPES = {
    TYPE_HV_POWER: "HvPower",
    TYPE_AUTO_BRIGHTNESS_NIXIE: "AutoBrightnessNixie",
    TYPE_AUTO_BRIGHTNESS_RGB: "AutoBrightnessRgb",
    TYPE_HOURLY_CHIME: "HourlyChime",
    TYPE_SHOW_LEADING_ZEROS: "ShowLeadingZeros",
    TYPE_STATUS_LED_AS_AM_PM: "StatusLedAsAmPm",
    TYPE_ALARM_BEEP: "AlarmBeep",
    TYPE_ALARM_BLINK: "AlarmBlink",
}

_ALARM_SLOT_SCHEMA = {cv.Required(CONF_SLOT): cv.int_range(min=1, max=8)}


def _switch_schema(base_schema):
    return base_schema.extend({cv.GenerateID(CONF_TUBE_CLOCK_ID): cv.use_id(TubeClock)})


def _alarm_switch_schema(base_schema):
    return _switch_schema(base_schema).extend(_ALARM_SLOT_SCHEMA)


CONFIG_SCHEMA = cv.typed_schema(
    {
        TYPE_HV_POWER: _switch_schema(
            switch.switch_schema(TubeClockSwitch, entity_category=ENTITY_CATEGORY_CONFIG)
        ),
        TYPE_AUTO_BRIGHTNESS_NIXIE: _switch_schema(
            switch.switch_schema(TubeClockSwitch, entity_category=ENTITY_CATEGORY_CONFIG)
        ),
        TYPE_AUTO_BRIGHTNESS_RGB: _switch_schema(
            switch.switch_schema(TubeClockSwitch, entity_category=ENTITY_CATEGORY_CONFIG)
        ),
        TYPE_HOURLY_CHIME: _switch_schema(
            switch.switch_schema(TubeClockSwitch, entity_category=ENTITY_CATEGORY_CONFIG)
        ),
        TYPE_SHOW_LEADING_ZEROS: _switch_schema(
            switch.switch_schema(TubeClockSwitch, entity_category=ENTITY_CATEGORY_CONFIG)
        ),
        TYPE_STATUS_LED_AS_AM_PM: _switch_schema(
            switch.switch_schema(TubeClockSwitch, entity_category=ENTITY_CATEGORY_CONFIG)
        ),
        TYPE_ALARM_BEEP: _alarm_switch_schema(
            switch.switch_schema(TubeClockSwitch, entity_category=ENTITY_CATEGORY_CONFIG)
        ),
        TYPE_ALARM_BLINK: _alarm_switch_schema(
            switch.switch_schema(TubeClockSwitch, entity_category=ENTITY_CATEGORY_CONFIG)
        ),
    },
    key=CONF_TYPE,
)


async def to_code(config):
    """Generate code for tube_clock switch."""
    var = await switch.new_switch(config)
    await cg.register_component(var, config)

    parent = await cg.get_variable(config[CONF_TUBE_CLOCK_ID])
    cg.add(var.set_parent(parent))
    cg.add(var.set_type(SWITCH_TYPES[config[CONF_TYPE]]))
    if CONF_SLOT in config:
        cg.add(var.set_slot(config[CONF_SLOT]))
