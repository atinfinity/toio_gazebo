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
Simulated battery for the toio cube (``/toio/battery_state``).

The real cube reports its state of charge on ``/toio/battery_state`` and
toio_fleet_adapter feeds it to Open-RMF, which is what makes RMF's automatic
``ChargeBattery`` task fire (send a low robot to its charger). Gazebo has no
battery, so without this node the reported SoC stays at 1.0 and ChargeBattery
never triggers in simulation.

This node models a simple battery so the same behaviour can be exercised in
sim: the charge drops while the cube runs and rises while it sits on its
charger, published as ``sensor_msgs/BatteryState`` on the same topic the real
cube uses. It is **opt-in** (``publish_battery:=true``); when off the topic is
not published and the simulation behaves exactly as before (SoC pinned at 1.0).

Discharge / charge
------------------
- Discharge at ``discharge_rate`` [SoC/s] while moving (a recent, non-zero
  ``cmd_vel``), otherwise at ``idle_discharge_rate``.
- Charge at ``charge_rate`` [SoC/s] while "on a charger":
  - If ``chargers`` is given (``"x y radius,..."`` in the ``map`` frame), the
    cube is charging when its ``map -> <frame_prefix>center`` pose is within a
    charger radius and it is not moving (pose-based, precise).
  - Otherwise a fallback treats the cube as charging once it has been
    stationary for ``charge_after_idle_sec`` (self-contained; the cube only
    idles that long at its charger, since holds elsewhere are short).
- ``quantize_steps`` > 0 rounds the reported SoC to that many steps
  (``10`` mirrors the real cube's 10 % increments); 0 reports the raw value.
"""

import math

from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import BatteryState
from tf2_ros import Buffer, TransformListener


def parse_chargers(text):
    """Parse ``"x y r, x y r"`` into a list of (x, y, r) tuples."""
    chargers = []
    for group in text.split(','):
        parts = group.split()
        if len(parts) == 3:
            chargers.append((float(parts[0]), float(parts[1]), float(parts[2])))
    return chargers


class BatteryModel:
    """Pure state-of-charge model, independent of ROS (unit-testable)."""

    def __init__(self, initial_soc=1.0, discharge_rate=0.01,
                 idle_discharge_rate=0.001, charge_rate=0.05, quantize_steps=0):
        self.soc = max(0.0, min(1.0, initial_soc))
        self.discharge_rate = discharge_rate
        self.idle_discharge_rate = idle_discharge_rate
        self.charge_rate = charge_rate
        self.quantize_steps = quantize_steps

    def step(self, dt, moving, charging):
        """Advance the SoC by ``dt`` seconds and return the new raw SoC."""
        if dt < 0:
            dt = 0.0
        if charging:
            self.soc = min(1.0, self.soc + self.charge_rate * dt)
        else:
            rate = self.discharge_rate if moving else self.idle_discharge_rate
            self.soc = max(0.0, self.soc - rate * dt)
        return self.soc

    def reported(self):
        """Return the charge to report, quantized when quantize_steps is set."""
        if self.quantize_steps > 0:
            return round(self.soc * self.quantize_steps) / self.quantize_steps
        return self.soc


class ToioBatteryNode(Node):
    def __init__(self):
        super().__init__('toio_battery')

        self.declare_parameter('initial_soc', 1.0)
        self.declare_parameter('discharge_rate', 0.01)        # SoC/s while moving
        self.declare_parameter('idle_discharge_rate', 0.001)  # SoC/s while idle
        self.declare_parameter('charge_rate', 0.05)           # SoC/s while charging
        self.declare_parameter('quantize_steps', 0)           # 0 = off, 10 = 10 % steps
        self.declare_parameter('chargers', '')                # "x y r,..." in map frame
        self.declare_parameter('charge_after_idle_sec', 10.0)
        self.declare_parameter('frame_prefix', '')
        self.declare_parameter('publish_rate', 5.0)

        self._model = BatteryModel(
            initial_soc=float(self.get_parameter('initial_soc').value),
            discharge_rate=float(self.get_parameter('discharge_rate').value),
            idle_discharge_rate=float(
                self.get_parameter('idle_discharge_rate').value),
            charge_rate=float(self.get_parameter('charge_rate').value),
            quantize_steps=int(self.get_parameter('quantize_steps').value))
        self._chargers = parse_chargers(
            str(self.get_parameter('chargers').value))
        self._charge_after_idle = float(
            self.get_parameter('charge_after_idle_sec').value)
        prefix = str(self.get_parameter('frame_prefix').value)
        self._center_frame = f'{prefix}center'
        rate = float(self.get_parameter('publish_rate').value)

        self._last_cmd = (0.0, 0.0)
        self._last_cmd_time = None
        self._idle_since = None
        self._last_tick = None

        # tf2 only needed for pose-based charger detection
        self._tf_buffer = None
        if self._chargers:
            self._tf_buffer = Buffer()
            TransformListener(self._tf_buffer, self)

        self.create_subscription(Twist, 'cmd_vel', self._on_cmd_vel, 10)
        self._pub = self.create_publisher(BatteryState, 'toio/battery_state', 10)
        self._timer = self.create_timer(1.0 / rate, self._tick)

    def _on_cmd_vel(self, msg):
        self._last_cmd = (msg.linear.x, msg.angular.z)
        self._last_cmd_time = self.get_clock().now()

    def _is_moving(self, now):
        if self._last_cmd_time is None:
            return False
        # a cmd_vel older than 0.5 s means the controller stopped commanding
        if (now - self._last_cmd_time).nanoseconds > 0.5e9:
            return False
        lin, ang = self._last_cmd
        return abs(lin) > 0.005 or abs(ang) > 0.02

    def _at_charger(self):
        if not self._chargers or self._tf_buffer is None:
            return False
        try:
            tr = self._tf_buffer.lookup_transform('map', self._center_frame,
                                                  Time())
        except Exception:
            return False
        x = tr.transform.translation.x
        y = tr.transform.translation.y
        for cx, cy, r in self._chargers:
            if math.hypot(x - cx, y - cy) <= r:
                return True
        return False

    def _charging(self, now, moving):
        if moving:
            return False
        if self._chargers:
            return self._at_charger()
        # fallback: charging once stationary long enough
        if self._idle_since is None:
            return False
        return (now - self._idle_since).nanoseconds >= self._charge_after_idle * 1e9

    def _tick(self):
        now = self.get_clock().now()
        if self._last_tick is None:
            self._last_tick = now
        dt = (now - self._last_tick).nanoseconds / 1e9
        self._last_tick = now

        moving = self._is_moving(now)
        if moving:
            self._idle_since = None
        elif self._idle_since is None:
            self._idle_since = now

        charging = self._charging(now, moving)
        self._model.step(dt, moving, charging)
        self._publish(charging)

    def _publish(self, charging):
        msg = BatteryState()
        msg.header.stamp = self.get_clock().now().to_msg()
        soc = self._model.reported()
        msg.percentage = soc
        msg.present = True
        if charging and soc < 1.0:
            msg.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_CHARGING
        elif soc >= 1.0:
            msg.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_FULL
        else:
            msg.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_DISCHARGING
        self._pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ToioBatteryNode()
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
