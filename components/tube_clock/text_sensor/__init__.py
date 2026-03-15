import esphome.codegen as cg
from esphome.components import text_sensor
import esphome.config_validation as cv
from esphome.const import CONF_TYPE, ENTITY_CATEGORY_DIAGNOSTIC

from .. import TubeClock, tube_clock_ns

DEPENDENCIES = ["tube_clock"]

TubeClockTextSensor = tube_clock_ns.class_(
    "TubeClockTextSensor", text_sensor.TextSensor, cg.Component
)

CONF_TUBE_CLOCK_ID = "tube_clock_id"

TYPE_FIRMWARE_VERSION = "firmware_version"

TEXT_SENSOR_TYPES = {
    TYPE_FIRMWARE_VERSION: tube_clock_ns.enum("TEXT_SENSOR_TYPE_FIRMWARE_VERSION"),
}


def _text_sensor_schema(base_schema):
    return base_schema.extend({cv.GenerateID(CONF_TUBE_CLOCK_ID): cv.use_id(TubeClock)})


CONFIG_SCHEMA = cv.typed_schema(
    {
        TYPE_FIRMWARE_VERSION: _text_sensor_schema(
            text_sensor.text_sensor_schema(
                TubeClockTextSensor,
                entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
            )
        ),
    },
    key=CONF_TYPE,
)


async def to_code(config):
    """Generate code for tube_clock text_sensor."""
    var = await text_sensor.new_text_sensor(config)
    await cg.register_component(var, config)

    parent = await cg.get_variable(config[CONF_TUBE_CLOCK_ID])
    cg.add(var.set_parent(parent))
    cg.add(var.set_type(TEXT_SENSOR_TYPES[config[CONF_TYPE]]))
