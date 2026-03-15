import esphome.codegen as cg
from esphome.components import select
import esphome.config_validation as cv
from esphome.const import CONF_TYPE, ENTITY_CATEGORY_CONFIG

from .. import TubeClock, tube_clock_ns

DEPENDENCIES = ["tube_clock"]

TubeClockSelect = tube_clock_ns.class_("TubeClockSelect", select.Select, cg.Component)

CONF_TUBE_CLOCK_ID = "tube_clock_id"

# Select types
TYPE_OPERATING_MODE = "operating_mode"
TYPE_DISPLAY_MODE = "display_mode"
TYPE_TEMPERATURE_SOURCE = "temperature_source"
TYPE_COLON_BEHAVIOR = "colon_behavior"
TYPE_DATE_FORMAT = "date_format"
TYPE_TIME_DISPLAY_MODE = "time_display_mode"
TYPE_TEMPERATURE_DISPLAY_UNITS = "temperature_display_units"

SELECT_TYPES = {
    TYPE_OPERATING_MODE: "OperatingMode",
    TYPE_DISPLAY_MODE: "DisplayMode",
    TYPE_TEMPERATURE_SOURCE: "TemperatureSource",
    TYPE_COLON_BEHAVIOR: "ColonBehavior",
    TYPE_DATE_FORMAT: "DateFormat",
    TYPE_TIME_DISPLAY_MODE: "TimeDisplayMode",
    TYPE_TEMPERATURE_DISPLAY_UNITS: "TemperatureDisplayUnits",
}


def _select_schema(base_schema):
    return base_schema.extend({cv.GenerateID(CONF_TUBE_CLOCK_ID): cv.use_id(TubeClock)})


CONFIG_SCHEMA = cv.typed_schema(
    {
        TYPE_OPERATING_MODE: _select_schema(select.select_schema(TubeClockSelect)),
        TYPE_DISPLAY_MODE: _select_schema(select.select_schema(TubeClockSelect)),
        TYPE_TEMPERATURE_SOURCE: _select_schema(
            select.select_schema(TubeClockSelect, entity_category=ENTITY_CATEGORY_CONFIG)
        ),
        TYPE_COLON_BEHAVIOR: _select_schema(
            select.select_schema(TubeClockSelect, entity_category=ENTITY_CATEGORY_CONFIG)
        ),
        TYPE_DATE_FORMAT: _select_schema(
            select.select_schema(TubeClockSelect, entity_category=ENTITY_CATEGORY_CONFIG)
        ),
        TYPE_TIME_DISPLAY_MODE: _select_schema(
            select.select_schema(TubeClockSelect, entity_category=ENTITY_CATEGORY_CONFIG)
        ),
        TYPE_TEMPERATURE_DISPLAY_UNITS: _select_schema(
            select.select_schema(TubeClockSelect, entity_category=ENTITY_CATEGORY_CONFIG)
        ),
    },
    key=CONF_TYPE,
)


async def to_code(config):
    """Generate code for tube_clock select."""
    # Get options based on type - these match the C++ implementation
    select_type = config[CONF_TYPE]
    options = []

    if select_type == TYPE_OPERATING_MODE:
        options = [
            "Main Menu", "Fixed Display", "Toggle Display", "Timer/Counter", "DMX512 Display",
            "Set Clock", "Set Date", "Set Timer", "System Status", "System Options",
            "Slot Beep", "Slot Blink", "Slot On/Off", "PM Indicator RGB Config", "Duration Clock",
            "Duration Date", "Duration Temp", "Duration Fade", "DST Begin Month", "DST Begin Week Ordinal",
            "DST End Month", "DST End Week Ordinal", "DST Switch DOW", "DST Switch Hour", "Effect Duration",
            "Effect Frequency", "Minimum Intensity", "Beeper Volume", "Temp Calibration", "Idle Timeout",
            "Date Format", "Time Zone", "Colon Behavior", "DMX512 Address", "Slot1 Time",
            "Slot2 Time", "Slot3 Time", "Slot4 Time", "Slot5 Time", "Slot6 Time",
            "Slot7 Time", "Slot8 Time",
        ]
    elif select_type == TYPE_DISPLAY_MODE:
        options = ["0/Time", "1/Time in Seconds", "2/Date", "3/Temperature", "4", "5", "6", "7", "8", "9"]
    elif select_type == TYPE_TEMPERATURE_SOURCE:
        options = ["STM32 Internal", "DS3234 RTC", "DS1722", "LM74", "External Serial"]
    elif select_type == TYPE_COLON_BEHAVIOR:
        options = ["On", "Off", "Blink", "Blink Upper", "Blink Lower", "Alternate Upper/Lower"]
    elif select_type == TYPE_DATE_FORMAT:
        options = ["YY-MM-DD", "DD-MM-YY", "MM-DD-YY"]
    elif select_type == TYPE_TIME_DISPLAY_MODE:
        options = ["24-Hour", "12-Hour"]
    elif select_type == TYPE_TEMPERATURE_DISPLAY_UNITS:
        options = ["Celsius", "Fahrenheit"]

    var = await select.new_select(config, options=options)
    await cg.register_component(var, config)

    parent = await cg.get_variable(config[CONF_TUBE_CLOCK_ID])
    cg.add(var.set_parent(parent))
    cg.add(var.set_type(SELECT_TYPES[select_type]))
