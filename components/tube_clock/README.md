# Tube Clock Component

ESPHome component for controlling and monitoring a Nixie Tube Clock via the Serial Remote Control API.

## Overview

This component provides complete integration with the Nixie Tube Clock project, enabling remote control and monitoring of all major clock functions through ESPHome. It communicates with the clock's STM32 microcontroller via UART using the Tube Clock Serial Remote Control API protocol.

## Hardware Connection

Connect your ESP32/ESP8266/RP2040 to the Tube Clock using UART:

- **Recommended**: Connect ESP TX to USART3 RX (PB10) and ESP RX to USART4 TX (PA0)
- **Alternative**: Connect to USART1 (TX: PB6, RX: PA10) - shared with GPS

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
  tx_pin: GPIO1
  rx_pin: GPIO3
  baud_rate: 115200

tube_clock:
  id: my_tube_clock
```

### Configuration Options

| Option | Required | Description |
|---|---|---|
| `uart_id` | Yes | Reference to the UART component used for normal communication |
| `boot_0_pin` | No | GPIO pin connected to the STM32 BOOT0 line (active-high) |
| `reset_pin` | No | GPIO pin connected to the STM32 NRST line (active-low pulse) |
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

Pin reconfiguration (`tx_pin` / `rx_pin`) is only supported on ESP32 with the ESP-IDF framework, as it relies on the ESP32 GPIO matrix.

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

```yaml
uart:
  tx_pin: GPIO1
  rx_pin: GPIO3
  baud_rate: 115200

tube_clock:
  id: my_tube_clock
  on_message:
    - logger.log: "Message received from clock"
  on_key_press:
    - logger.log:
        format: "Key pressed: 0x%02X"
        args: ["key"]
  on_key_release:
    - logger.log:
        format: "Key released: 0x%02X"
        args: ["key"]
  on_mode_change:
    - logger.log:
        format: "Mode: %u, Submode: %u"
        args: ["mode", "submode"]

# Select entities for mode control
select:
  - platform: tube_clock
    tube_clock_id: my_tube_clock
    name: "Display Mode"
    type: operating_mode

  - platform: tube_clock
    tube_clock_id: my_tube_clock
    name: "Temperature Source"
    type: temperature_source

  - platform: tube_clock
    tube_clock_id: my_tube_clock
    name: "Date Format"
    type: date_format

# Binary sensors for keys and GPS
binary_sensor:
  - platform: tube_clock
    tube_clock_id: my_tube_clock
    name: "Key Up"
    type: key_up

  - platform: tube_clock
    tube_clock_id: my_tube_clock
    name: "GPS Fix Valid"
    type: gps_fix_valid

# Sensors for ADC and temperature readings
sensor:
  - platform: tube_clock
    tube_clock_id: my_tube_clock
    name: "Ambient Light"
    type: adc_light

  - platform: tube_clock
    tube_clock_id: my_tube_clock
    name: "Internal Temperature"
    type: temp_stm32

  - platform: tube_clock
    tube_clock_id: my_tube_clock
    name: "RTC Temperature"
    type: temp_ds3234

  - platform: tube_clock
    tube_clock_id: my_tube_clock
    name: "Battery Voltage"
    type: adc_vbatt

# Switches for system control
switch:
  - platform: tube_clock
    tube_clock_id: my_tube_clock
    name: "High Voltage Power"
    type: hv_power

  - platform: tube_clock
    tube_clock_id: my_tube_clock
    name: "Auto Brightness"
    type: auto_brightness

  - platform: tube_clock
    tube_clock_id: my_tube_clock
    name: "Hourly Chime"
    type: hourly_chime

# Light entity for display intensity
light:
  - platform: tube_clock
    tube_clock_id: my_tube_clock
    name: "Display Brightness"

# Number entities for settings
number:
  - platform: tube_clock
    tube_clock_id: my_tube_clock
    name: "Fade Duration"
    type: fade_duration

  - platform: tube_clock
    tube_clock_id: my_tube_clock
    name: "Minimum Intensity"
    type: minimum_intensity

  - platform: tube_clock
    tube_clock_id: my_tube_clock
    name: "Beeper Volume"
    type: beeper_volume
```

## Platform Components

### Select Platform

Control operating modes and display settings.

**Available types:**
- `operating_mode` - Main operating mode selection
- `fixed_display_mode` - Fixed display mode (Time/Date/Temp)
- `toggle_display_mode` - Auto-toggle display mode
- `temperature_source` - Active temperature sensor
- `colon_behavior` - Colon display behavior
- `date_format` - Date format (MM/DD/YYYY, DD/MM/YYYY, YYYY-MM-DD)
- `display_12_hour` - 12/24 hour format toggle
- `display_fahrenheit` - Celsius/Fahrenheit toggle

### Binary Sensor Platform

Monitor key states and GPS status.

**Available types:**
- `key_up`, `key_down`, `key_enter`, `key_a`, `key_b`, `key_c` - Capacitive touch keys
- `gps_fix_valid` - GPS fix status

### Sensor Platform

Read ADC values and temperatures.

**Available types:**
- `adc_light` - Ambient light level (raw ADC)
- `adc_vdda` - VddA supply voltage
- `adc_vbatt` - Battery voltage
- `temp_stm32` - STM32 internal temperature
- `temp_ds3234` - DS3234 RTC temperature
- `temp_ds1722` - DS1722 temperature sensor
- `temp_lm74` - LM74 temperature sensor

### Switch Platform

Control system features.

**Available types:**
- `hv_power` - High-voltage power supply
- `auto_brightness` - Automatic brightness control
- `hourly_chime` - Hourly chime enable

### Light Platform

Control display brightness (0-255).

### Number Platform

Configure timing and calibration values.

**Available types:**
- `time_display_duration` - Time display duration (deciseconds)
- `date_display_duration` - Date display duration (deciseconds)
- `temp_display_duration` - Temperature display duration (deciseconds)
- `fade_duration` - Digit fade duration (milliseconds)
- `effect_duration` - Display effect duration (milliseconds)
- `effect_frequency` - Display effect frequency (milliseconds)
- `minimum_intensity` - Minimum brightness level
- `beeper_volume` - Beeper volume level
- `temp_cal_stm32` - STM32 temperature calibration offset
- `temp_cal_ds3234` - DS3234 temperature calibration offset
- `temp_cal_ds1722` - DS1722 temperature calibration offset
- `temp_cal_lm74` - LM74 temperature calibration offset

## Actions

### Set Temperature

Set the external temperature value (overrides all hardware sensors).

```yaml
button:
  - platform: template
    name: "Set External Temperature"
    on_press:
      - tube_clock.set_temperature:
          id: my_tube_clock
          temperature: 23.5
```

### Set Time and Date

Set the clock's time and date.

```yaml
button:
  - platform: template
    name: "Sync Clock Time"
    on_press:
      - tube_clock.set_time:
          id: my_tube_clock
          hours: 12
          minutes: 30
          seconds: 0
          year: 2026
          month: 2
          day: 19
```

## Automation Triggers

### on_message

Triggered when any message is received from the clock.

```yaml
tube_clock:
  id: my_tube_clock
  on_message:
    - logger.log:
        format: "Received: %s"
        args: ["message.c_str()"]
```

### on_key_press / on_key_release

Triggered when keys are pressed or released.

```yaml
tube_clock:
  id: my_tube_clock
  on_key_press:
    - lambda: |-
        ESP_LOGI("tube_clock", "Key 0x%02X pressed", key);
```

### on_mode_change

Triggered when the operating mode changes.

```yaml
tube_clock:
  id: my_tube_clock
  on_mode_change:
    - logger.log:
        format: "Mode: %u, Submode: %u"
        args: ["mode", "submode"]
```

## Protocol Details

The component implements the Tube Clock Serial Remote Control API using NMEA-style ASCII sentences:

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

- [Tube Clock Serial API Documentation](https://github.com/kbx81/TubeClock/blob/main/docs/SERIAL_API.md)
- [ESPHome UART Documentation](https://esphome.io/components/uart.html)
