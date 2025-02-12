# kbx's ESPHome External Components

Here you'll find external components I've hacked together for use with [ESPHome](https://esphome.io).

## Usage

To use components you find here in your own configuration, you'll need to add a few lines to your device's YAML configuration, similar to the following example:

```yaml
external_components:
  - source:
      type: git
      url: https://github.com/kbx81/esphome_external_components
    components:
      - esp32_rmt_dop_led_h_bridge
      - remote_base
```

Please see ESPHome's [external components](https://esphome.io/components/external_components.html) documentation for more detail.

If you found any of this helpful and feel so inclined, please [Buy Me A Coffee](https://bmc.link/kbx81)! ☕️