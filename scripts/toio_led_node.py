#!/usr/bin/env python3
# Copyright 2026
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""
Drive the simulated indicator from the timed and patterned LED topics.

The indicator in Gazebo is a system plugin that takes a plain color on a
gz-transport topic, with the lighting time fixed for the whole world in SDF.
The real cube instead takes the time per command (``toio/led_timed``) and can
be handed a whole blink sequence to play by itself (``toio/led_pattern``).

ros_gz_bridge only carries message types it knows, so toio_msgs cannot reach
the plugin. This node closes the gap on the ROS side: it keeps the timing
itself and publishes plain colors to ``toio/led``, which is already bridged.

That is a real difference from hardware, not just an implementation detail.
On the cube the sequence runs in firmware, so it survives a BLE dropout and
its timing does not depend on the host. Here the host drives every step, so a
busy or paused simulation stretches the pattern. Nothing in Gazebo can play a
sequence on its own, so this is as close as the simulation gets.
"""

import rclpy
from rclpy.node import Node

from std_msgs.msg import ColorRGBA
from toio_msgs.msg import Led, LedPattern

OFF = (0.0, 0.0, 0.0)


def clamp_duration_ms(duration_ms):
    """Clip a lighting time the way the cube does."""
    return min(int(duration_ms), Led.DURATION_MAX_MS)


def pattern_steps(msg):
    """
    Expand a LedPattern into the (color, seconds) steps to play in order.

    Returns None when the cube would reject the message, so the caller can
    warn instead of playing something the real cube never would.
    """
    if not msg.steps:
        return None
    if len(msg.steps) > LedPattern.STEPS_MAX:
        return None

    once = [
        ((step.color.r, step.color.g, step.color.b),
         clamp_duration_ms(step.duration_ms) / 1000.0)
        for step in msg.steps
    ]
    # repeat 0 is "until the next indicator command" on the cube. A timer here
    # can be cancelled by the next command, so it is honoured as an endless
    # loop rather than being flattened to a single pass.
    if msg.repeat == LedPattern.REPEAT_INFINITE:
        return once
    return once * msg.repeat


class ToioLedNode(Node):
    """Turn timed colors and blink patterns into plain colors over time."""

    def __init__(self):
        super().__init__('toio_led')

        # Relative topic names so the cubes of a multi-robot simulation are
        # separated by the namespace of the node, as on the real cube
        self._color_pub = self.create_publisher(ColorRGBA, 'toio/led', 10)
        self.create_subscription(Led, 'toio/led_timed', self._on_timed, 10)
        self.create_subscription(
            LedPattern, 'toio/led_pattern', self._on_pattern, 10)

        self._timer = None
        self._steps = []
        self._index = 0
        self._loop = False

    # ------------------------------------------------------------------
    # Commands
    # ------------------------------------------------------------------
    def _on_timed(self, msg):
        self._cancel()
        color = (msg.color.r, msg.color.g, msg.color.b)
        self._publish(color)

        seconds = clamp_duration_ms(msg.duration_ms) / 1000.0
        if msg.duration_ms < Led.DURATION_MIN_MS:
            # Under the minimum the cube treats it as "no time limit"
            self.get_logger().debug('lit until the next command')
            return
        self.get_logger().debug(f'lit for {seconds:.3f}s')
        self._steps = [(OFF, 0.0)]
        self._index = 0
        self._loop = False
        self._timer = self.create_timer(seconds, self._advance)

    def _on_pattern(self, msg):
        steps = pattern_steps(msg)
        if steps is None:
            self.get_logger().warning(
                f'led pattern with {len(msg.steps)} steps is not one the '
                f'cube would accept, ignoring it')
            return

        self._cancel()
        self._steps = steps
        self._index = 0
        self._loop = msg.repeat == LedPattern.REPEAT_INFINITE
        self.get_logger().info(
            f'playing a {len(steps)} step pattern'
            + (' until the next command' if self._loop else ''))
        self._play_step()

    # ------------------------------------------------------------------
    # Sequencing
    # ------------------------------------------------------------------
    def _play_step(self):
        color, seconds = self._steps[self._index]
        self._publish(color)
        self._timer = self.create_timer(max(seconds, 0.001), self._advance)

    def _advance(self):
        self._cancel_timer()
        self._index += 1
        if self._index >= len(self._steps):
            if not self._loop:
                # A pattern that ran out leaves the indicator off, the same as
                # the last step of a finite pattern on the cube
                self._publish(OFF)
                self._steps = []
                return
            self._index = 0
        self._play_step()

    def _publish(self, color):
        msg = ColorRGBA()
        msg.r, msg.g, msg.b = [float(c) for c in color]
        msg.a = 1.0
        self._color_pub.publish(msg)

    def _cancel(self):
        self._cancel_timer()
        self._steps = []
        self._index = 0
        self._loop = False

    def _cancel_timer(self):
        timer = self._timer
        self._timer = None
        if timer is not None:
            timer.cancel()
            self.destroy_timer(timer)


def main(args=None):
    rclpy.init(args=args)
    node = ToioLedNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
