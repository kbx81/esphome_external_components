import esphome.codegen as cg
from esphome.components import light
import esphome.config_validation as cv
from esphome.const import CONF_OUTPUT_ID

from .. import TubeClock, tube_clock_ns

DEPENDENCIES = ["tube_clock"]

CONF_TUBE_CLOCK_ID = "tube_clock_id"
CONF_TYPE = "type"
TYPE_NIXIE = "nixie"
TYPE_RGB = "rgb"

TubeClockLightOutput = tube_clock_ns.class_("TubeClockLightOutput", light.LightOutput)
TubeClockRGBLightOutput = tube_clock_ns.class_("TubeClockRGBLightOutput", light.LightOutput)

_COMMON = {
    cv.GenerateID(CONF_TUBE_CLOCK_ID): cv.use_id(TubeClock),
}

CONFIG_SCHEMA = cv.typed_schema(
    {
        TYPE_NIXIE: light.BRIGHTNESS_ONLY_LIGHT_SCHEMA.extend(
            {
                cv.GenerateID(CONF_OUTPUT_ID): cv.declare_id(TubeClockLightOutput),
                **_COMMON,
            }
        ),
        TYPE_RGB: light.RGB_LIGHT_SCHEMA.extend(
            {
                cv.GenerateID(CONF_OUTPUT_ID): cv.declare_id(TubeClockRGBLightOutput),
                **_COMMON,
            }
        ),
    },
    key=CONF_TYPE,
)


async def to_code(config):
    """Generate code for tube_clock light."""
    parent = await cg.get_variable(config[CONF_TUBE_CLOCK_ID])
    var = cg.new_Pvariable(config[CONF_OUTPUT_ID], parent)
    await light.register_light(var, config)
