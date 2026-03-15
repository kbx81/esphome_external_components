import esphome.codegen as cg
from esphome.components import binary_sensor
import esphome.config_validation as cv
from esphome.const import CONF_TYPE, ENTITY_CATEGORY_DIAGNOSTIC

from .. import TubeClock, tube_clock_ns

DEPENDENCIES = ["tube_clock"]

TubeClockBinarySensor = tube_clock_ns.class_(
    "TubeClockBinarySensor", binary_sensor.BinarySensor, cg.Component
)

CONF_TUBE_CLOCK_ID = "tube_clock_id"
CONF_KEYS = "keys"

# Key names mapped to their bitmask values (matching TubeClockKey enum in tube_clock.h)
KEY_BITMASKS = {
    "key_up": 0x01,
    "key_down": 0x02,
    "key_enter": 0x04,
    "key_c": 0x08,
    "key_b": 0x10,
    "key_a": 0x20,
}

# Binary sensor types
TYPE_KEY = "key"
TYPE_GPS_CONNECTED = "gps_connected"
TYPE_GPS_FIX_VALID = "gps_fix_valid"
TYPE_RTTTL_PLAYING = "rtttl_playing"
TYPE_TIMER_ALARM_ACTIVE = "timer_alarm_active"


def _binary_sensor_schema(base_schema):
    return base_schema.extend({cv.GenerateID(CONF_TUBE_CLOCK_ID): cv.use_id(TubeClock)})


CONFIG_SCHEMA = cv.typed_schema(
    {
        TYPE_KEY: _binary_sensor_schema(
            binary_sensor.binary_sensor_schema(
                TubeClockBinarySensor, entity_category=ENTITY_CATEGORY_DIAGNOSTIC
            ).extend(
                {
                    cv.Required(CONF_KEYS): cv.All(
                        cv.ensure_list(cv.one_of(*KEY_BITMASKS, lower=True)),
                        cv.Length(min=1),
                    ),
                }
            )
        ),
        TYPE_GPS_CONNECTED: _binary_sensor_schema(
            binary_sensor.binary_sensor_schema(TubeClockBinarySensor, entity_category=ENTITY_CATEGORY_DIAGNOSTIC)
        ),
        TYPE_GPS_FIX_VALID: _binary_sensor_schema(
            binary_sensor.binary_sensor_schema(TubeClockBinarySensor, entity_category=ENTITY_CATEGORY_DIAGNOSTIC)
        ),
        TYPE_RTTTL_PLAYING: _binary_sensor_schema(
            binary_sensor.binary_sensor_schema(TubeClockBinarySensor)
        ),
        TYPE_TIMER_ALARM_ACTIVE: _binary_sensor_schema(
            binary_sensor.binary_sensor_schema(TubeClockBinarySensor)
        ),
    },
    key=CONF_TYPE,
)


async def to_code(config):
    """Generate code for tube_clock binary_sensor."""
    var = await binary_sensor.new_binary_sensor(config)
    await cg.register_component(var, config)

    parent = await cg.get_variable(config[CONF_TUBE_CLOCK_ID])
    cg.add(var.set_parent(parent))

    sensor_type = config[CONF_TYPE]
    if sensor_type == TYPE_KEY:
        key_mask = 0
        for key_name in config[CONF_KEYS]:
            key_mask |= KEY_BITMASKS[key_name]
        cg.add(var.set_key_mask(key_mask))
    elif sensor_type == TYPE_GPS_CONNECTED:
        cg.add(var.set_gps_connected())
    elif sensor_type == TYPE_GPS_FIX_VALID:
        cg.add(var.set_gps_fix_valid())
    elif sensor_type == TYPE_RTTTL_PLAYING:
        cg.add(var.set_rtttl_playing())
    elif sensor_type == TYPE_TIMER_ALARM_ACTIVE:
        cg.add(var.set_timer_alarm_active())
