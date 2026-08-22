#!/usr/bin/env python3
# Copyright (C) 2026 atinfinity
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
Stub of the toio cube's motion detection (``/toio/motion``).

This mirrors the ``/toio/motion`` interface of the real cube (toio_ros2) so
that the same subscribers work against the simulation, but the values are a
fixed stub: the sim cube is always level and upright on its wheels, and it
never reports a collision, a double tap or a shake. Gazebo has no equivalent
of the cube's motion-detection notification, and the events that would drive
these fields (a knock, a tap, a shake) are not modelled, so publishing a
constant, clearly-documented stub is preferred over faking events.

- ``horizontal``: always ``True`` (cube level and on its wheels)
- ``collision``: always ``False``
- ``double_tap``: always ``False``
- ``posture``: always ``POSTURE_TOP`` (upright)
- ``shake``: always ``SHAKE_NONE``

``collision_threshold`` and ``horizontal_threshold`` are declared to match the
toio_ros2 parameters but have no effect here.
"""

import rclpy
from rclpy.node import Node

from toio_msgs.msg import MotionDetection


def build_motion_stub():
    """Return the fixed MotionDetection published in simulation (no stamp)."""
    msg = MotionDetection()
    msg.horizontal = True
    msg.collision = False
    msg.double_tap = False
    msg.posture = MotionDetection.POSTURE_TOP
    msg.shake = MotionDetection.SHAKE_NONE
    return msg


class ToioMotionNode(Node):
    def __init__(self):
        super().__init__('toio_motion')

        # Declared for interface parity with toio_ros2; unused by this stub.
        self.declare_parameter('collision_threshold', 7)
        self.declare_parameter('horizontal_threshold', 45)

        self._pub = self.create_publisher(MotionDetection, 'toio/motion', 10)
        # The real cube publishes on change and once on connection. Here the
        # value never changes, so it is republished at a low rate so that a
        # late subscriber still gets it.
        self._timer = self.create_timer(1.0, self._publish)
        self._publish()

    def _publish(self):
        msg = build_motion_stub()
        msg.header.stamp = self.get_clock().now().to_msg()
        self._pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ToioMotionNode()
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
