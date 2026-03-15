import esphome.codegen as cg
from esphome.components import datetime as dt
import esphome.config_validation as cv
from esphome.const import CONF_ENTITY_CATEGORY, ENTITY_CATEGORY_CONFIG

from .. import TubeClock, tube_clock_ns

DEPENDENCIES = ["tube_clock"]

CONF_TUBE_CLOCK_ID = "tube_clock_id"
CONF_SLOT = "slot"

TubeClockDatetime = tube_clock_ns.class_("TubeClockDatetime", dt.TimeEntity, cg.Component)

CONFIG_SCHEMA = (
    dt.time_schema(TubeClockDatetime)
    .extend(
        {
            cv.GenerateID(CONF_TUBE_CLOCK_ID): cv.use_id(TubeClock),
            cv.Required(CONF_SLOT): cv.int_range(min=1, max=8),
            cv.Optional(CONF_ENTITY_CATEGORY, default=ENTITY_CATEGORY_CONFIG): cv.entity_category,
        }
    )
    .extend(cv.COMPONENT_SCHEMA)
)


async def to_code(config):
    """Generate code for tube_clock datetime."""
    var = await dt.new_datetime(config)
    await cg.register_component(var, config)

    parent = await cg.get_variable(config[CONF_TUBE_CLOCK_ID])
    cg.add(var.set_parent(parent))
    cg.add(var.set_slot(config[CONF_SLOT]))
