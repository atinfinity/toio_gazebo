# Multi-robot simulation

`toio_multi_simulation.launch.py` spawns two toio robots (`toio1` and `toio2`).

```bash
ros2 launch toio_gazebo toio_multi_simulation.launch.py
```

Each robot is separated by ROS namespace and TF frame prefix.

| robot | cmd_vel | joint_states | led | sound | TF |
| --- | --- | --- | --- | --- | --- |
| toio1 | `/toio1/cmd_vel` | `/toio1/joint_states` | `/toio1/toio/led` | `/toio1/toio/sound` | `map -> toio1/center` |
| toio2 | `/toio2/cmd_vel` | `/toio2/joint_states` | `/toio2/toio/led` | `/toio2/toio/sound` | `map -> toio2/center` |

An example to move each robot independently:

```bash
ros2 topic pub -r 10 /toio1/cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.05}}'
ros2 topic pub -r 10 /toio2/cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.05}}'
```

An example to light each robot in a different color:

```bash
ros2 topic pub --once /toio1/toio/led std_msgs/msg/ColorRGBA '{r: 1.0, g: 0.0, b: 0.0, a: 1.0}'
ros2 topic pub --once /toio2/toio/led std_msgs/msg/ColorRGBA '{r: 0.0, g: 0.0, b: 1.0, a: 1.0}'
```

To spawn an additional robot into a running simulation, use `spawn_toio.launch.py`:

```bash
ros2 launch toio_gazebo spawn_toio.launch.py namespace:=toio3 robot_name:=toio3 frame_prefix:=toio3/ x_pose:=0.145 y_pose:=-0.095
```
