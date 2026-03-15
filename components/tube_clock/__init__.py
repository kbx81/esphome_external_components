from esphome import automation, pins
import esphome.codegen as cg
from esphome.components import esp32, uart
from esphome.components.const import CONF_PARITY
import esphome.config_validation as cv
from esphome.const import (
    CONF_BAUD_RATE,
    CONF_DAY,
    CONF_HOUR,
    CONF_HOURS,
    CONF_ID,
    CONF_MINUTES,
    CONF_MONTH,
    CONF_RESET_PIN,
    CONF_RX_PIN,
    CONF_SECONDS,
    CONF_TEMPERATURE,
    CONF_TIME,
    CONF_TRIGGER_ID,
    CONF_TX_PIN,
    CONF_URL,
    CONF_YEAR,
)

CODEOWNERS = ["@kbx81"]
DEPENDENCIES = ["uart"]
MULTI_CONF = True

CONF_BOOT_0_PIN = "boot_0_pin"
CONF_DFU_UART_CONFIG = "dfu_uart_config"
CONF_ON_MESSAGE = "on_message"
CONF_RTTTL = "rtttl"

DFU_UART_CONFIG_SCHEMA = cv.Schema(
    {
        cv.Optional(CONF_TX_PIN): pins.internal_gpio_output_pin_number,
        cv.Optional(CONF_RX_PIN): pins.internal_gpio_input_pin_number,
        cv.Optional(CONF_BAUD_RATE): cv.positive_int,
        cv.Optional(CONF_PARITY, default="EVEN"): cv.enum(
            uart.UART_PARITY_OPTIONS, upper=True
        ),
    }
)

tube_clock_ns = cg.esphome_ns.namespace("tube_clock")
TubeClock = tube_clock_ns.class_("TubeClock", cg.Component, uart.UARTDevice)
TubeClockListener = tube_clock_ns.class_("TubeClockListener")

# Automation triggers
MessageTrigger = tube_clock_ns.class_(
    "MessageTrigger", automation.Trigger.template(cg.std_string)
)

# Actions
SetTemperatureAction = tube_clock_ns.class_("SetTemperatureAction", automation.Action)
SetTimeAction = tube_clock_ns.class_("SetTimeAction", automation.Action)
PlayRtttlAction = tube_clock_ns.class_("PlayRtttlAction", automation.Action)
StopRtttlAction = tube_clock_ns.class_("StopRtttlAction", automation.Action)
SaveSettingsAction = tube_clock_ns.class_("SaveSettingsAction", automation.Action)
FactoryResetAction = tube_clock_ns.class_("FactoryResetAction", automation.Action)
ResetAction = tube_clock_ns.class_("ResetAction", automation.Action)
EnterBootloaderAction = tube_clock_ns.class_("EnterBootloaderAction", automation.Action)
UpdateFirmwareAction = tube_clock_ns.class_("UpdateFirmwareAction", automation.Action)
TimerRunUpAction = tube_clock_ns.class_("TimerRunUpAction", automation.Action)
TimerRunDownAction = tube_clock_ns.class_("TimerRunDownAction", automation.Action)
TimerStopAction = tube_clock_ns.class_("TimerStopAction", automation.Action)
TimerResetAction = tube_clock_ns.class_("TimerResetAction", automation.Action)
TimerResetToAction = tube_clock_ns.class_("TimerResetToAction", automation.Action)
TimerClearAlarmAction = tube_clock_ns.class_("TimerClearAlarmAction", automation.Action)
PlayChimeAction = tube_clock_ns.class_("PlayChimeAction", automation.Action)

CONFIG_SCHEMA = (
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(TubeClock),
            cv.Optional(CONF_BOOT_0_PIN): pins.gpio_output_pin_schema,
            cv.Optional(CONF_DFU_UART_CONFIG): DFU_UART_CONFIG_SCHEMA,
            cv.Optional(CONF_RESET_PIN): pins.gpio_output_pin_schema,
            cv.Optional(CONF_ON_MESSAGE): automation.validate_automation(
                {
                    cv.GenerateID(CONF_TRIGGER_ID): cv.declare_id(MessageTrigger),
                }
            ),
        }
    )
    .extend(cv.COMPONENT_SCHEMA)
    .extend(uart.UART_DEVICE_SCHEMA)
)


async def to_code(config):
    """Generate code for the tube_clock component."""
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    await uart.register_uart_device(var, config)

    cg.add_define("USE_TUBE_CLOCK_DFU")
    esp32.include_builtin_idf_component("esp_http_client")
    esp32.add_idf_sdkconfig_option("CONFIG_ESP_TLS_INSECURE", True)
    esp32.add_idf_sdkconfig_option("CONFIG_ESP_TLS_SKIP_SERVER_CERT_VERIFY", True)

    if CONF_BOOT_0_PIN in config:
        pin = await cg.gpio_pin_expression(config[CONF_BOOT_0_PIN])
        cg.add(var.set_boot_0_pin(pin))

    if CONF_DFU_UART_CONFIG in config:
        dfu_cfg = config[CONF_DFU_UART_CONFIG]
        if CONF_TX_PIN in dfu_cfg:
            cg.add(var.set_dfu_tx_pin(dfu_cfg[CONF_TX_PIN]))
        if CONF_RX_PIN in dfu_cfg:
            cg.add(var.set_dfu_rx_pin(dfu_cfg[CONF_RX_PIN]))
        if CONF_BAUD_RATE in dfu_cfg:
            cg.add(var.set_dfu_baud_rate(dfu_cfg[CONF_BAUD_RATE]))
        cg.add(var.set_dfu_parity(dfu_cfg[CONF_PARITY]))

    if CONF_RESET_PIN in config:
        pin = await cg.gpio_pin_expression(config[CONF_RESET_PIN])
        cg.add(var.set_reset_pin(pin))

    for conf in config.get(CONF_ON_MESSAGE, []):
        trigger = cg.new_Pvariable(conf[CONF_TRIGGER_ID], var)
        await automation.build_automation(trigger, [(cg.std_string, "message")], conf)


# Action schemas
@automation.register_action(
    "tube_clock.set_temperature",
    SetTemperatureAction,
    cv.Schema(
        {
            cv.GenerateID(): cv.use_id(TubeClock),
            cv.Required(CONF_TEMPERATURE): cv.templatable(
                cv.float_range(min=-100.0, max=200.0)
            ),
        }
    ),
    synchronous=False,
)
async def tube_clock_set_temperature_to_code(config, action_id, template_arg, args):
    """Build set_temperature action."""
    paren = await cg.get_variable(config[CONF_ID])
    var = cg.new_Pvariable(action_id, template_arg, paren)
    template_ = await cg.templatable(config[CONF_TEMPERATURE], args, float)
    cg.add(var.set_temperature(template_))
    return var


_SET_TIME_INDIVIDUAL_FIELDS = [CONF_HOURS, CONF_MINUTES, CONF_SECONDS, CONF_YEAR, CONF_MONTH, CONF_DAY]


def _validate_set_time_schema(config):
    has_time = CONF_TIME in config
    has_fields = any(f in config for f in _SET_TIME_INDIVIDUAL_FIELDS)
    if has_time and has_fields:
        raise cv.Invalid(
            f"Cannot specify both '{CONF_TIME}' and individual time fields"
            f" ({', '.join(_SET_TIME_INDIVIDUAL_FIELDS)})"
        )
    if not has_time and not has_fields:
        raise cv.Invalid(
            f"Must specify either '{CONF_TIME}' or all individual time fields"
            f" ({', '.join(_SET_TIME_INDIVIDUAL_FIELDS)})"
        )
    if has_fields:
        for field in _SET_TIME_INDIVIDUAL_FIELDS:
            if field not in config:
                raise cv.Invalid(
                    f"When using individual time fields, '{field}' is required"
                )
    return config


@automation.register_action(
    "tube_clock.set_time",
    SetTimeAction,
    cv.All(
        cv.Schema(
            {
                cv.GenerateID(): cv.use_id(TubeClock),
                cv.Optional(CONF_TIME): cv.templatable(cv.returning_lambda),
                cv.Optional(CONF_HOURS): cv.templatable(cv.int_range(min=0, max=23)),
                cv.Optional(CONF_MINUTES): cv.templatable(cv.int_range(min=0, max=59)),
                cv.Optional(CONF_SECONDS): cv.templatable(cv.int_range(min=0, max=59)),
                cv.Optional(CONF_YEAR): cv.templatable(cv.int_range(min=2000, max=2099)),
                cv.Optional(CONF_MONTH): cv.templatable(cv.int_range(min=1, max=12)),
                cv.Optional(CONF_DAY): cv.templatable(cv.int_range(min=1, max=31)),
            }
        ),
        _validate_set_time_schema,
    ),
    synchronous=False,
)
async def tube_clock_set_time_to_code(config, action_id, template_arg, args):
    """Build set_time action."""
    paren = await cg.get_variable(config[CONF_ID])
    var = cg.new_Pvariable(action_id, template_arg, paren)

    if CONF_TIME in config:
        template_ = await cg.templatable(config[CONF_TIME], args, cg.ESPTime)
        cg.add(var.set_time(template_))
        cg.add(var.set_has_time(True))
    else:
        template_ = await cg.templatable(config[CONF_HOURS], args, int)
        cg.add(var.set_hours(template_))
        template_ = await cg.templatable(config[CONF_MINUTES], args, int)
        cg.add(var.set_minutes(template_))
        template_ = await cg.templatable(config[CONF_SECONDS], args, int)
        cg.add(var.set_seconds(template_))
        template_ = await cg.templatable(config[CONF_YEAR], args, int)
        cg.add(var.set_year(template_))
        template_ = await cg.templatable(config[CONF_MONTH], args, int)
        cg.add(var.set_month(template_))
        template_ = await cg.templatable(config[CONF_DAY], args, int)
        cg.add(var.set_day(template_))

    return var


@automation.register_action(
    "tube_clock.play_rtttl",
    PlayRtttlAction,
    cv.Schema(
        {
            cv.GenerateID(): cv.use_id(TubeClock),
            cv.Required(CONF_RTTTL): cv.templatable(
                cv.All(cv.string, cv.Length(min=1, max=251))
            ),
        }
    ),
    synchronous=False,
)
async def tube_clock_play_rtttl_to_code(config, action_id, template_arg, args):
    """Build play_rtttl action."""
    paren = await cg.get_variable(config[CONF_ID])
    var = cg.new_Pvariable(action_id, template_arg, paren)
    template_ = await cg.templatable(config[CONF_RTTTL], args, cg.std_string)
    cg.add(var.set_rtttl(template_))
    return var


@automation.register_action(
    "tube_clock.stop_rtttl",
    StopRtttlAction,
    cv.Schema(
        {
            cv.GenerateID(): cv.use_id(TubeClock),
        }
    ),
    synchronous=False,
)
async def tube_clock_stop_rtttl_to_code(config, action_id, template_arg, args):
    """Build stop_rtttl action."""
    paren = await cg.get_variable(config[CONF_ID])
    return cg.new_Pvariable(action_id, template_arg, paren)


@automation.register_action(
    "tube_clock.save_settings",
    SaveSettingsAction,
    cv.Schema(
        {
            cv.GenerateID(): cv.use_id(TubeClock),
        }
    ),
    synchronous=False,
)
async def tube_clock_save_settings_to_code(config, action_id, template_arg, args):
    """Build save_settings action."""
    paren = await cg.get_variable(config[CONF_ID])
    return cg.new_Pvariable(action_id, template_arg, paren)


@automation.register_action(
    "tube_clock.factory_reset",
    FactoryResetAction,
    cv.Schema(
        {
            cv.GenerateID(): cv.use_id(TubeClock),
        }
    ),
    synchronous=False,
)
async def tube_clock_factory_reset_to_code(config, action_id, template_arg, args):
    """Build factory_reset action."""
    paren = await cg.get_variable(config[CONF_ID])
    return cg.new_Pvariable(action_id, template_arg, paren)


@automation.register_action(
    "tube_clock.reset",
    ResetAction,
    cv.Schema(
        {
            cv.GenerateID(): cv.use_id(TubeClock),
        }
    ),
    synchronous=False,
)
async def tube_clock_reset_to_code(config, action_id, template_arg, args):
    """Build reset action."""
    paren = await cg.get_variable(config[CONF_ID])
    return cg.new_Pvariable(action_id, template_arg, paren)


@automation.register_action(
    "tube_clock.enter_bootloader",
    EnterBootloaderAction,
    cv.Schema(
        {
            cv.GenerateID(): cv.use_id(TubeClock),
        }
    ),
    synchronous=False,
)
async def tube_clock_enter_bootloader_to_code(config, action_id, template_arg, args):
    """Build enter_bootloader action."""
    paren = await cg.get_variable(config[CONF_ID])
    return cg.new_Pvariable(action_id, template_arg, paren)


@automation.register_action(
    "tube_clock.update_firmware",
    UpdateFirmwareAction,
    cv.Schema(
        {
            cv.GenerateID(): cv.use_id(TubeClock),
            cv.Required(CONF_URL): cv.templatable(cv.string),
        }
    ),
    synchronous=False,
)
async def tube_clock_update_firmware_to_code(config, action_id, template_arg, args):
    """Build update_firmware action."""
    paren = await cg.get_variable(config[CONF_ID])
    var = cg.new_Pvariable(action_id, template_arg, paren)
    template_ = await cg.templatable(config[CONF_URL], args, cg.std_string)
    cg.add(var.set_url(template_))
    return var


@automation.register_action(
    "tube_clock.timer_run_up",
    TimerRunUpAction,
    cv.Schema({cv.GenerateID(): cv.use_id(TubeClock)}),
    synchronous=False,
)
async def tube_clock_timer_run_up_to_code(config, action_id, template_arg, args):
    """Build timer_run_up action."""
    paren = await cg.get_variable(config[CONF_ID])
    return cg.new_Pvariable(action_id, template_arg, paren)


@automation.register_action(
    "tube_clock.timer_run_down",
    TimerRunDownAction,
    cv.Schema({cv.GenerateID(): cv.use_id(TubeClock)}),
    synchronous=False,
)
async def tube_clock_timer_run_down_to_code(config, action_id, template_arg, args):
    """Build timer_run_down action."""
    paren = await cg.get_variable(config[CONF_ID])
    return cg.new_Pvariable(action_id, template_arg, paren)


@automation.register_action(
    "tube_clock.timer_stop",
    TimerStopAction,
    cv.Schema({cv.GenerateID(): cv.use_id(TubeClock)}),
    synchronous=False,
)
async def tube_clock_timer_stop_to_code(config, action_id, template_arg, args):
    """Build timer_stop action."""
    paren = await cg.get_variable(config[CONF_ID])
    return cg.new_Pvariable(action_id, template_arg, paren)


@automation.register_action(
    "tube_clock.timer_reset",
    TimerResetAction,
    cv.Schema({cv.GenerateID(): cv.use_id(TubeClock)}),
    synchronous=False,
)
async def tube_clock_timer_reset_to_code(config, action_id, template_arg, args):
    """Build timer_reset action."""
    paren = await cg.get_variable(config[CONF_ID])
    return cg.new_Pvariable(action_id, template_arg, paren)


@automation.register_action(
    "tube_clock.timer_reset_to",
    TimerResetToAction,
    cv.Schema(
        {
            cv.GenerateID(): cv.use_id(TubeClock),
            cv.Required(CONF_SECONDS): cv.templatable(cv.int_range(min=0, max=999999)),
        }
    ),
    synchronous=False,
)
async def tube_clock_timer_reset_to_to_code(config, action_id, template_arg, args):
    """Build timer_reset_to action."""
    paren = await cg.get_variable(config[CONF_ID])
    var = cg.new_Pvariable(action_id, template_arg, paren)
    template_ = await cg.templatable(config[CONF_SECONDS], args, cg.uint32)
    cg.add(var.set_seconds(template_))
    return var


@automation.register_action(
    "tube_clock.timer_clear_alarm",
    TimerClearAlarmAction,
    cv.Schema({cv.GenerateID(): cv.use_id(TubeClock)}),
    synchronous=False,
)
async def tube_clock_timer_clear_alarm_to_code(config, action_id, template_arg, args):
    """Build timer_clear_alarm action."""
    paren = await cg.get_variable(config[CONF_ID])
    return cg.new_Pvariable(action_id, template_arg, paren)


@automation.register_action(
    "tube_clock.play_chime",
    PlayChimeAction,
    cv.Schema(
        {
            cv.GenerateID(): cv.use_id(TubeClock),
            cv.Optional(CONF_HOUR): cv.templatable(cv.int_range(min=0, max=23)),
        }
    ),
    synchronous=False,
)
async def tube_clock_play_chime_to_code(config, action_id, template_arg, args):
    """Build play_chime action."""
    paren = await cg.get_variable(config[CONF_ID])
    var = cg.new_Pvariable(action_id, template_arg, paren)
    if CONF_HOUR in config:
        cg.add(var.set_has_hour(True))
        template_ = await cg.templatable(config[CONF_HOUR], args, int)
        cg.add(var.set_hour(template_))
    return var
