# toio_ros2 interface support

`toio_gazebo` aims for interface parity with the real cube provided by
[toio_ros2](https://github.com/atinfinity/toio_ros2), so the same topics and
actions work against the simulation and against the hardware. Some parts of the
cube have no equivalent in Gazebo, so they are stubbed or left out.

This page lists **every** ROS 2 interface of `toio_ros2_node`
([toio_ros2 interfaces](https://github.com/atinfinity/toio_ros2/blob/main/docs/interfaces.md))
and its status in the simulation. For usage and examples of the supported ones,
see [Topics and launch arguments](topics.md).

## Legend

| mark | meaning |
| --- | --- |
| ✅ | Supported — same topic name and type, behaves like the cube |
| ⚠️ | Stub — the topic exists for parity but carries a fixed / simplified value |
| ❌ | Not simulated — no equivalent in Gazebo; the topic is not present |

## Subscribed topics (commands to the cube)

| toio_ros2 topic | type | sim | how / notes |
| --- | --- | :---: | --- |
| `/cmd_vel` | geometry_msgs/msg/Twist | ✅ | Driven by the Gazebo `DiffDrive` plugin (bridged through `ros_gz_bridge`). The hardware-only guards `stop_on_position_id_missed` / `stop_on_button` do not apply |
| `/toio/led` | std_msgs/msg/ColorRGBA | ✅ | Rendered by the `ToioLedSystem` Gazebo plugin (color bridged as `gz.msgs.Color`) |
| `/toio/sound` | std_msgs/msg/UInt8 | ✅ | Played on the host by `toio_sound_node` (Gazebo has no audio output) |
| `/toio/led_timed` | toio_msgs/msg/Led | ✅ | `toio_led_node` keeps the lighting time on the ROS side and drives `/toio/led` |
| `/toio/led_pattern` | toio_msgs/msg/LedPattern | ✅ | `toio_led_node` sequences the blink pattern (timing depends on the sim clock; `repeat` 0 loops) |
| `/toio/melody` | toio_msgs/msg/Melody | ✅ | Played by `toio_sound_node` on the host (`repeat` 0 plays once — a handed-off clip cannot be stopped) |
| `/goal_pose` | geometry_msgs/msg/PoseStamped | ❌ | The cube's built-in target motion is not simulated. In sim an external planner (Nav2 / Open-RMF) owns every motion through `/cmd_vel` |

## Published topics (cube state)

| toio_ros2 topic | type | sim | how / notes |
| --- | --- | :---: | --- |
| `/tf` | tf2_msgs/msg/TFMessage | ✅ | `map -> odom -> center` (or `map -> center` with `publish_odom:=False`). The cube pose comes from Gazebo ground truth, so `map -> center` is exact |
| `/odom` | nav_msgs/msg/Odometry | ✅ | Wheel odometry at 20 Hz (`odom -> center`), only when `publish_odom` is true. Mirrors toio_ros2's `/odom` |
| `/toio/imu` | sensor_msgs/msg/Imu | ✅ | From a Gazebo IMU sensor on the `center` link, every `imu_interval_ms`. Unlike the real cube (orientation only), angular velocity and linear acceleration are passed through as-is |
| `/toio/motion` | toio_msgs/msg/MotionDetection | ⚠️ | **Stub.** Gazebo has no motion-detection equivalent, so a fixed value is published for parity: `horizontal` true, `posture` `POSTURE_TOP`, `collision` / `double_tap` / `shake` false / none |
| `/toio/pose` | geometry_msgs/msg/PoseStamped | ❌ | Not published as a topic. The pose is available as the `map -> center` TF (ground truth); consumers that need it (e.g. the Open-RMF fleet adapter in sim) read it from TF |
| `/toio/position_id_missed` | std_msgs/msg/Bool | ❌ | No mat / Position ID reading in sim, so there is nothing to miss |
| `/toio/battery_state` | sensor_msgs/msg/BatteryState | ❌ | No battery model in sim |
| `/toio/button` | std_msgs/msg/Bool | ❌ | No button in sim |
| `/diagnostics` | diagnostic_msgs/msg/DiagnosticArray | ❌ | The `diagnostic_updater` health reporting of the hardware node is not reproduced |

## Action servers

| toio_ros2 action | type | sim | how / notes |
| --- | --- | :---: | --- |
| `/dock_to_pose` | nav2_msgs/action/NavigateToPose | ❌ | Precise docking uses the cube's built-in target motion, which is not simulated. Approach a waypoint through Nav2 / `/cmd_vel` instead |

## Services

`toio_ros2_node` exposes **no services**, so there is nothing to simulate here.

## Simulation-only additions

Topics that exist in the simulation but are not part of the toio_ros2 interface.

| topic | type | notes |
| --- | --- | --- |
| `/joint_states` | sensor_msgs/msg/JointState | Wheel joint states from the Gazebo `JointStatePublisher`, used by `robot_state_publisher` for the cube's visual TF. The real cube does not expose wheel encoders as `/joint_states` |

## Notes

- All names are shown without a namespace. Under `toio_multi_simulation` each
  cube's topics are namespaced (`/toio1/...`, `/toio2/...`) and `frame_prefix`
  is applied to the TF frames, exactly as with `toio_multi_bringup` on hardware.
- The command-side launch arguments (`led_duration_ms`, `led_light_intensity`,
  `sound_volume`, `imu_interval_ms`, `publish_odom`) are parameters of the
  toio_ros2 node on hardware; in the simulation they are launch arguments
  because the LED is a Gazebo plugin and the sound is a separate node. See
  [Topics and launch arguments](topics.md).
- The hardware-only parameters that guard motion (`stop_on_position_id_missed`,
  `stop_on_button`, `collision_threshold`, `horizontal_threshold`,
  `enable_goal_pose_motion`, the `field_*` / `goal_*` / `dock_*` set) have no
  effect in the simulation; some are accepted and ignored for parity.
