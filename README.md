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
      - tube_clock
```

Please see ESPHome's [external components](https://esphome.io/components/external_components.html) documentation for more detail.

I hope you find my work useful. If you do and want to support me, please check out the links below.

[![Sponsor me on GitHub](https://img.shields.io/badge/sponsor%20me%20on%20GitHub-sponsor-green.svg)](https://github.com/sponsors/kbx81)
[![Support Nabu Casa](https://img.shields.io/badge/Nabu%20Casa-support-navy.svg)](https://nabucasa.com)
[![Support the Open Home Foundation](https://img.shields.io/badge/Open%20Home%20Foundation-support-blue.svg)](https://openhomefoundation.org)
