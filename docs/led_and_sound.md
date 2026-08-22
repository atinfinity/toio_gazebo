# Notes on the LED and the sound effects

The lamp is the ball on the underside of the cube, as
[the specification](https://toio.github.io/toio-spec/en/docs/ble_light/)
describes it, and is driven by the `ToioLedSystem` Gazebo plugin of this
package. Only the cap below the flat underside is visible, so it is best seen
from a low angle.

The plugin drives a light along with the visual, so the mat picks up the color
of the lamp rather than the lamp only lighting up itself. As on the real cube,
the mat right under it takes a strong spot about the size of the lamp, which
fades out over roughly the size of the cube. Set `led_light_intensity:=0` to
leave the mat alone.
The way the plugin drives the material and the light through
`components::VisualCmd` and `components::LightCmd` follows
[gz_sim_led_plugin](https://github.com/jasmeet0915/gz_sim_led_plugin)
(Apache License 2.0).

Gazebo has no audio output, so `toio_sound_node.py` plays the sound effects on
the host through `aplay`. If `aplay` is missing, the requests are only logged.
The sound effects of the real cube cannot be redistributed here, so each id is
approximated by a synthesized motif rather than the original sound.
