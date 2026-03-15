# Tube Clock Component

ESPHome component for controlling and monitoring kbx's Tube Clock via its [Serial Remote Control API](https://github.com/kbx81/TubeClock/blob/master/docs/SERIAL_API.md).

## Overview

This component provides complete integration with kbx's Tube Clock project, enabling remote control and monitoring of all major clock functions through ESPHome. It communicates with the clock's STM32 microcontroller via a serial connection using the Tube Clock's [Serial Remote Control API](https://github.com/kbx81/TubeClock/blob/master/docs/SERIAL_API.md).

See [https://github.com/kbx81/TubeClock](https://github.com/kbx81/TubeClock) for more information about the Tube Clock project.

## Hardware Connection

- **Recommended**: Connect ESP TX to USART3 RX (PB10) and ESP RX to USART4 TX (PA0)
- **Alternative**: Connect to USART1 (TX: PB6, RX: PA10) — shared with the tube clock's on-board GPS

### UART Settings

- Baud rate: 115200
- Data bits: 8
- Parity: None
- Stop bits: 1
- Logic level: 3.3V

## Configuration

### Basic Configuration

```yaml
uart:
  id: uart_clock
  tx_pin: GPIO20
  rx_pin: GPIO7
  baud_rate: 115200

tube_clock:
  id: my_tube_clock
  uart_id: uart_clock
```

### Configuration Options

| Option | Required | Description |
|---|---|---|
| `uart_id` | Yes | Reference to the UART component used for normal communication |
| `reset_pin` | No | GPIO pin connected to the STM32 NRST line (active-low pulse) |
| `boot_0_pin` | No | GPIO pin connected to the STM32 BOOT0 line (active-high); required for DFU |
| `dfu_uart_config` | No | UART reconfiguration settings applied during firmware updates (see below) |
| `on_message` | No | Automation trigger fired for every raw message received from the clock |

#### `dfu_uart_config`

When performing a firmware update (DFU), the component temporarily reconfigures the primary UART to match the STM32 bootloader's protocol requirements, then restores the original settings afterward. `dfu_uart_config` lets you override any of the parameters used during DFU:

| Option | Default | Description |
|---|---|---|
| `tx_pin` | *(original pin)* | TX pin to use during DFU (ESP32/IDF only) |
| `rx_pin` | *(original pin)* | RX pin to use during DFU (ESP32/IDF only) |
| `baud_rate` | *(original baud rate)* | Baud rate during DFU |
| `parity` | `EVEN` | Parity during DFU (`NONE`, `EVEN`, or `ODD`) |

All fields are optional. If `dfu_uart_config` is omitted entirely, the UART is still reconfigured to even parity for DFU and restored to `NONE` afterward (STM32 bootloader requires 8E1).

Pin reconfiguration (`tx_pin` / `rx_pin`) is only supported on ESP32 with the ESP-IDF framework, as it relies on the ESP32's GPIO matrix.

Example:
```yaml
uart:
  id: uart_clock
  tx_pin: GPIO20
  rx_pin: GPIO7
  baud_rate: 115200

tube_clock:
  id: my_tube_clock
  uart_id: uart_clock
  boot_0_pin: GPIO3
  reset_pin: GPIO5
  dfu_uart_config:
    tx_pin: GPIO4    # Different wiring for DFU path
    rx_pin: GPIO9
    baud_rate: 460800
    parity: EVEN
```

### Full Configuration Example

See [`example.yaml`](example.yaml) for a complete, annotated configuration covering all available platforms and actions.

## Platform Components

### Select Platform

Control operating modes and display settings.

**Available types:**

| Type | Description |
|---|---|
| `operating_mode` | Top-level operating mode (time, date, temperature, timer, etc.) |
| `display_mode` | Active display slot when `operating_mode` is "Fixed Display" |
| `temperature_source` | Which hardware sensor to use for the temperature display slot |
| `date_format` | Date format shown on the tubes (e.g. YY-MM-DD, DD-MM-YY, MM-DD-YY) |
| `colon_behavior` | Behavior of the colon separator LEDs between digit pairs |
| `time_display_mode` | 12-hour vs. 24-hour time display |
| `temperature_display_units` | Temperature display units (Celsius / Fahrenheit) |

### Binary Sensor Platform

Monitor key states and GPS/alarm status.

**Available types:**

| Type | Description |
|---|---|
| `key` | True while the specified key(s) are held; use `keys:` to list one or more key names. Valid key names: `key_up`, `key_down`, `key_enter`, `key_a`, `key_b`, `key_c`. When multiple keys are listed, the sensor is true only when all of them are held simultaneously (chord). |
| `gps_connected` | True when a GPS receiver is electrically connected |
| `gps_fix_valid` | True when the GPS receiver has acquired a valid position fix |
| `rtttl_playing` | True while the beeper is playing an RTTTL melody |
| `timer_alarm_active` | True while a timer alarm is active (timer elapsed and not yet cleared) |

### Sensor Platform

Read ADC values and temperatures.

**Available types:**

| Type | Description |
|---|---|
| `adc_light` | Ambient light level read by the onboard ADC (used for auto-brightness) |
| `adc_vdda` | MCU supply voltage (VDDA rail) |
| `adc_vbatt` | Coin-cell backup battery voltage |
| `gps_sats` | Number of GPS satellites currently tracked |
| `temp_stm32` | STM32 internal die temperature |
| `temp_ds3234` | DS3234 RTC temperature register (±0.25 °C accuracy) |
| `temp_ds1722` | DS1722 digital temperature sensor (optional external sensor) |
| `temp_lm74` | LM74 digital temperature sensor (optional external sensor) |

### Switch Platform

Control system features.

**Available types:**

| Type | Description |
|---|---|
| `hv_power` | Enable/disable the high-voltage supply for the Nixie tubes |
| `auto_brightness_nixie` | Automatically dim Nixie tubes based on the ambient light ADC reading |
| `auto_brightness_rgb` | Automatically dim the RGB status LED based on the ambient light ADC reading |
| `show_leading_zeros` | Show leading zeros on the hours digit (e.g. 09:05 instead of 9:05) |
| `status_led_as_am_pm` | Use the PM indicator LED to signal AM vs. PM in 12-hour mode |
| `hourly_chime` | Play a chime on the beeper at the top of each hour |
| `alarm_beep` | Enable audible beep when an alarm slot triggers; requires `slot: 1`–`8` |
| `alarm_blink` | Enable tube blinking when an alarm slot triggers; requires `slot: 1`–`8` |

### Light Platform

Control display and status LED brightness/color.

**Available types:**

| Type | Description |
|---|---|
| `nixie` | Nixie tube brightness (brightness-only, 0–100%). Ignored when Auto Brightness (Nixie) switch is on. |
| `rgb` | RGB status LED (full color control). Ignored when Auto Brightness (RGB LED) switch is on. |

### Text Sensor Platform

**Available types:**

| Type | Description |
|---|---|
| `firmware_version` | Clock MCU firmware version string (e.g. "26.03.01 (build 150)") |

### Number Platform

Configure timing and calibration values.

**Available types:**

| Type | Description |
|---|---|
| `time_display_duration` | How long (seconds) the time display is shown before cycling to the next slot |
| `date_display_duration` | How long (seconds) the date display is shown |
| `temp_display_duration` | How long (seconds) the temperature display is shown |
| `fade_duration` | Duration of the cross-fade animation between digit changes (milliseconds) |
| `effect_duration` | Duration of anti-cathode-poisoning slot effects (milliseconds) |
| `effect_frequency` | How often (seconds) the anti-poisoning slot effect is triggered |
| `minimum_intensity` | Floor brightness used by the auto-brightness algorithm (1–1000) |
| `beeper_volume` | Beeper volume (0 = off, 7 = loudest) |
| `temp_cal_stm32` | Temperature offset applied to STM32 internal sensor readings (±9.9 °C) |
| `temp_cal_ds3234` | Temperature offset applied to DS3234 RTC sensor readings (±9.9 °C) |
| `temp_cal_ds1722` | Temperature offset applied to DS1722 sensor readings (±9.9 °C) |
| `temp_cal_lm74` | Temperature offset applied to LM74 sensor readings (±9.9 °C) |
| `timer_reset_value` | Value the timer reloads when `tube_clock.timer_reset` is called (seconds, 0–65535) |
| `idle_timeout` | Inactivity timeout before the display returns to the main clock (seconds, 10–600) |

### Datetime Platform

The clock has 8 independent alarm time slots (slot: 1–8). Each is exposed as a `time` entity in Home Assistant.

```yaml
datetime:
  - platform: tube_clock
    tube_clock_id: my_tube_clock
    name: "Alarm 1"
    slot: 1
```

Use the `alarm_beep` and `alarm_blink` switches to control what happens when each alarm fires.

## Actions

### `tube_clock.save_settings`

Persist all current settings to the clock's non-volatile memory. Settings are lost on power-cycle if this is not called after making changes.

### `tube_clock.factory_reset`

Reset all settings back to factory defaults and trigger a soft reset of the clock MCU.

### `tube_clock.reset`

Soft-reset the clock MCU (equivalent to pressing the hardware reset button).

### `tube_clock.set_time`

Sync the clock's time from an ESPHome time source or explicit field values.

```yaml
# From a time source component:
- tube_clock.set_time:
    id: my_tube_clock
    time: !lambda "return id(homeassistant_time).now();"

# With explicit fields (all required):
- tube_clock.set_time:
    id: my_tube_clock
    hours: 12
    minutes: 0
    seconds: 0
    year: 2025
    month: 6
    day: 1
```

### `tube_clock.set_temperature`

Push an external temperature value to the clock (degrees Celsius). Used when `temperature_source` is set to "External Serial".

```yaml
- tube_clock.set_temperature:
    id: my_tube_clock
    temperature: !lambda "return 23.5;"
```

### `tube_clock.play_rtttl`

Play an RTTTL melody on the beeper.

```yaml
- tube_clock.play_rtttl:
    id: my_tube_clock
    rtttl: "Scale:d=4,o=5,b=120:c,d,e,f,g,a,b,c6"
```

### `tube_clock.stop_rtttl`

Stop any currently playing RTTTL melody immediately.

### `tube_clock.play_chime`

Play the hourly chime. Optionally specify which hour's chime pattern (0–23); without `hour`, plays the chime for the current hour.

```yaml
- tube_clock.play_chime:
    id: my_tube_clock
    # hour: 12  # Optional
```

### Timer Actions

The clock has a built-in countdown/count-up timer.

| Action | Description |
|---|---|
| `tube_clock.timer_run_up` | Start counting up |
| `tube_clock.timer_run_down` | Start counting down |
| `tube_clock.timer_stop` | Stop the timer |
| `tube_clock.timer_reset` | Reset the timer to the value set by the `timer_reset_value` number entity |
| `tube_clock.timer_reset_to` | Reset the timer to a specific value in seconds (0–999999); requires `seconds:` |
| `tube_clock.timer_clear_alarm` | Clear a latched timer alarm so the clock resumes normal display |

### Firmware Update Actions

| Action | Description |
|---|---|
| `tube_clock.update_firmware` | Trigger an OTA firmware update of the clock MCU from a remote URL. Requires `boot_0_pin` and `reset_pin` to be configured. Requires `url:` pointing to a raw STM32 binary (.bin) file. |
| `tube_clock.enter_bootloader` | Enter the STM32 bootloader manually (for use with external DFU tools). Requires `boot_0_pin` to be configured. |

## Automation Triggers

### `on_message`

Triggered when any message is received from the clock. The `message` variable is a `std::string` containing the raw message.

```yaml
tube_clock:
  id: my_tube_clock
  on_message:
    - logger.log:
        format: "Clock message: %s"
        args: ["message.c_str()"]
```

## Protocol Details

The component implements the Tube Clock [Serial Remote Control API](https://github.com/kbx81/TubeClock/blob/master/docs/SERIAL_API.md) using NMEA-style ASCII sentences:

```
$TC{C|S}<category><action>[<data>]*XX\n
```

- Commands (`C`) are sent from ESP to clock
- Status (`S`) messages are received from clock (responses and notifications)
- All messages include XOR checksum validation
- The clock sends unsolicited notifications for state changes

## Notes

- Temperature values are in tenths of degrees Celsius (×10) in the protocol but automatically converted to degrees for ESPHome sensors
- Voltage values are in millivolts in the protocol but converted to volts for ESPHome sensors
- The unavailable temperature sentinel value (32767) is automatically handled as NaN in ESPHome
- The component caches all state locally and updates platform entities via listeners

## See Also

- [Tube Clock Serial API Documentation](https://github.com/kbx81/TubeClock/blob/master/docs/SERIAL_API.md)
- [ESPHome UART Documentation](https://esphome.io/components/uart.html)
