import esphome.codegen as cg
from esphome.components import sensor
import esphome.config_validation as cv
from esphome.const import (
    CONF_TYPE,
    DEVICE_CLASS_ILLUMINANCE,
    DEVICE_CLASS_TEMPERATURE,
    DEVICE_CLASS_VOLTAGE,
    ENTITY_CATEGORY_DIAGNOSTIC,
    STATE_CLASS_MEASUREMENT,
    UNIT_CELSIUS,
    UNIT_VOLT,
)

from .. import TubeClock, tube_clock_ns

DEPENDENCIES = ["tube_clock"]

TubeClockSensor = tube_clock_ns.class_("TubeClockSensor", sensor.Sensor, cg.Component)

CONF_TUBE_CLOCK_ID = "tube_clock_id"

# Sensor types
TYPE_ADC_LIGHT = "adc_light"
TYPE_ADC_VDDA = "adc_vdda"
TYPE_ADC_VBATT = "adc_vbatt"
TYPE_GPS_SATS = "gps_sats"
TYPE_TEMP_STM32 = "temp_stm32"
TYPE_TEMP_DS3234 = "temp_ds3234"
TYPE_TEMP_DS1722 = "temp_ds1722"
TYPE_TEMP_LM74 = "temp_lm74"

SENSOR_TYPES = {
    TYPE_ADC_LIGHT: "AdcLight",
    TYPE_ADC_VDDA: "AdcVdda",
    TYPE_ADC_VBATT: "AdcVbatt",
    TYPE_GPS_SATS: "GpsSats",
    TYPE_TEMP_STM32: "TempStm32",
    TYPE_TEMP_DS3234: "TempDs3234",
    TYPE_TEMP_DS1722: "TempDs1722",
    TYPE_TEMP_LM74: "TempLm74",
}

def _sensor_schema(base_schema):
    return base_schema.extend({cv.GenerateID(CONF_TUBE_CLOCK_ID): cv.use_id(TubeClock)})


CONFIG_SCHEMA = cv.typed_schema(
    {
        TYPE_ADC_LIGHT: _sensor_schema(
            sensor.sensor_schema(
                TubeClockSensor,
                device_class=DEVICE_CLASS_ILLUMINANCE,
                state_class=STATE_CLASS_MEASUREMENT,
            )
        ),
        TYPE_ADC_VDDA: _sensor_schema(
            sensor.sensor_schema(
                TubeClockSensor,
                unit_of_measurement=UNIT_VOLT,
                accuracy_decimals=3,
                device_class=DEVICE_CLASS_VOLTAGE,
                state_class=STATE_CLASS_MEASUREMENT,
            )
        ),
        TYPE_ADC_VBATT: _sensor_schema(
            sensor.sensor_schema(
                TubeClockSensor,
                unit_of_measurement=UNIT_VOLT,
                accuracy_decimals=3,
                device_class=DEVICE_CLASS_VOLTAGE,
                state_class=STATE_CLASS_MEASUREMENT,
            )
        ),
        TYPE_GPS_SATS: _sensor_schema(
            sensor.sensor_schema(
                TubeClockSensor,
                accuracy_decimals=0,
                entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
                state_class=STATE_CLASS_MEASUREMENT,
            )
        ),
        TYPE_TEMP_STM32: _sensor_schema(
            sensor.sensor_schema(
                TubeClockSensor,
                unit_of_measurement=UNIT_CELSIUS,
                accuracy_decimals=1,
                device_class=DEVICE_CLASS_TEMPERATURE,
                state_class=STATE_CLASS_MEASUREMENT,
            )
        ),
        TYPE_TEMP_DS3234: _sensor_schema(
            sensor.sensor_schema(
                TubeClockSensor,
                unit_of_measurement=UNIT_CELSIUS,
                accuracy_decimals=1,
                device_class=DEVICE_CLASS_TEMPERATURE,
                state_class=STATE_CLASS_MEASUREMENT,
            )
        ),
        TYPE_TEMP_DS1722: _sensor_schema(
            sensor.sensor_schema(
                TubeClockSensor,
                unit_of_measurement=UNIT_CELSIUS,
                accuracy_decimals=1,
                device_class=DEVICE_CLASS_TEMPERATURE,
                state_class=STATE_CLASS_MEASUREMENT,
            )
        ),
        TYPE_TEMP_LM74: _sensor_schema(
            sensor.sensor_schema(
                TubeClockSensor,
                unit_of_measurement=UNIT_CELSIUS,
                accuracy_decimals=1,
                device_class=DEVICE_CLASS_TEMPERATURE,
                state_class=STATE_CLASS_MEASUREMENT,
            )
        ),
    },
    key=CONF_TYPE,
)


async def to_code(config):
    """Generate code for tube_clock sensor."""
    var = await sensor.new_sensor(config)
    await cg.register_component(var, config)

    parent = await cg.get_variable(config[CONF_TUBE_CLOCK_ID])
    cg.add(var.set_parent(parent))
    cg.add(var.set_type(SENSOR_TYPES[config[CONF_TYPE]]))
