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

import os
import tempfile

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnShutdown
from launch.substitutions import Command, LaunchConfiguration

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def launch_setup(context, *args, **kwargs):
    namespace = LaunchConfiguration('namespace').perform(context)
    robot_name = LaunchConfiguration('robot_name').perform(context)
    frame_prefix = LaunchConfiguration('frame_prefix').perform(context)
    robot_sdf = LaunchConfiguration('robot_sdf').perform(context)
    led_duration_ms = LaunchConfiguration('led_duration_ms').perform(context)
    led_light_intensity = LaunchConfiguration('led_light_intensity').perform(context)
    sound_volume = int(LaunchConfiguration('sound_volume').perform(context))
    imu_interval_ms = LaunchConfiguration('imu_interval_ms').perform(context)
    publish_odom = LaunchConfiguration('publish_odom').perform(
        context).lower() in ('true', '1')
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_simulator = LaunchConfiguration('use_simulator')
    use_robot_state_pub = LaunchConfiguration('use_robot_state_pub')
    pose = {'x': LaunchConfiguration('x_pose', default='0.00'),
            'y': LaunchConfiguration('y_pose', default='0.00'),
            'z': LaunchConfiguration('z_pose', default='0.01'),
            'R': LaunchConfiguration('roll', default='0.00'),
            'P': LaunchConfiguration('pitch', default='0.00'),
            'Y': LaunchConfiguration('yaw', default='0.00')}

    ns = '' if namespace in ('', '/') else '/' + namespace.strip('/')

    # The bridge config of ros_gz_bridge is not affected by the node
    # namespace, so a config file with fully resolved topic names is
    # generated per robot instance.
    bridge_config = f"""\
- ros_topic_name: "{ns}/cmd_vel"
  gz_topic_name: "/model/{robot_name}/cmd_vel"
  ros_type_name: "geometry_msgs/msg/Twist"
  gz_type_name: "gz.msgs.Twist"
  direction: ROS_TO_GZ

- ros_topic_name: "{ns}/joint_states"
  gz_topic_name: "/model/{robot_name}/joint_states"
  ros_type_name: "sensor_msgs/msg/JointState"
  gz_type_name: "gz.msgs.Model"
  direction: GZ_TO_ROS

- ros_topic_name: "/tf"
  gz_topic_name: "/model/{robot_name}/pose"
  ros_type_name: "tf2_msgs/msg/TFMessage"
  gz_type_name: "gz.msgs.Pose_V"
  direction: GZ_TO_ROS

- ros_topic_name: "{ns}/toio/led"
  gz_topic_name: "/model/{robot_name}/led"
  ros_type_name: "std_msgs/msg/ColorRGBA"
  gz_type_name: "gz.msgs.Color"
  direction: ROS_TO_GZ

- ros_topic_name: "{ns}/toio/imu"
  gz_topic_name: "/model/{robot_name}/imu"
  ros_type_name: "sensor_msgs/msg/Imu"
  gz_type_name: "gz.msgs.IMU"
  direction: GZ_TO_ROS
"""
    # Wheel odometry, mirroring toio_ros2 /odom (odom -> center). The tf tree
    # (map -> odom -> center) is set up in simulation.launch with map -> center
    # kept at the mat-perfect ground-truth pose, so the /odom message pose is
    # the wheel integration while the tf localisation stays ground truth.
    if publish_odom:
        bridge_config += f"""
- ros_topic_name: "{ns}/odom"
  gz_topic_name: "/model/{robot_name}/odom"
  ros_type_name: "nav_msgs/msg/Odometry"
  gz_type_name: "gz.msgs.Odometry"
  direction: GZ_TO_ROS
"""
    bridge_config_file = tempfile.mktemp(
        prefix=f'toio_bridge_{robot_name}_', suffix='.yaml')
    with open(bridge_config_file, 'w') as f:
        f.write(bridge_config)

    remove_temp_bridge_file = RegisterEventHandler(event_handler=OnShutdown(
        on_shutdown=[
            OpaqueFunction(function=lambda _: os.remove(bridge_config_file))
        ]))

    start_robot_state_publisher_cmd = Node(
        condition=IfCondition(use_robot_state_pub),
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        namespace=namespace,
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time,
             'frame_prefix': frame_prefix,
             # Declared as a string because the description is otherwise
             # parsed as YAML, which any colon in the URDF would break.
             'robot_description': ParameterValue(
                 Command(
                     ['xacro', ' ', robot_sdf, ' ', 'robot_name:=', robot_name,
                      ' ', 'led_duration_ms:=', led_duration_ms,
                      ' ', 'led_light_intensity:=', led_light_intensity,
                      ' ', 'frame_prefix:=', frame_prefix,
                      ' ', 'imu_interval_ms:=', imu_interval_ms]),
                 value_type=str)}
        ],
    )

    # The indicator plugin takes a plain color with the lighting time fixed in
    # SDF, so the per-command time and the blink patterns are sequenced here
    # and published as plain colors (see toio_led_node.py).
    led_node = Node(
        package='toio_gazebo',
        executable='toio_led_node.py',
        name='toio_led',
        namespace=namespace,
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
    )

    # Gazebo has no audio output, so the sound effects are played on the host
    # instead of by a Gazebo plugin.
    sound_node = Node(
        package='toio_gazebo',
        executable='toio_sound_node.py',
        name='toio_sound',
        namespace=namespace,
        output='screen',
        parameters=[
            {
                'sound_volume': sound_volume,
                'use_sim_time': use_sim_time,
            }
        ],
    )

    # Gazebo has no equivalent of the cube's motion-detection notification, so
    # this node publishes a fixed stub on /toio/motion for interface parity
    # (see toio_motion_node.py).
    motion_node = Node(
        package='toio_gazebo',
        executable='toio_motion_node.py',
        name='toio_motion',
        namespace=namespace,
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
    )

    # Gazebo has no battery, so an opt-in node models the state of charge and
    # publishes it on /toio/battery_state (the topic the real cube and
    # toio_fleet_adapter use) so RMF's ChargeBattery can fire in simulation.
    # Off by default: without it the reported SoC stays 1.0, as before.
    battery_node = Node(
        condition=IfCondition(LaunchConfiguration('publish_battery')),
        package='toio_gazebo',
        executable='toio_battery_node.py',
        name='toio_battery',
        namespace=namespace,
        output='screen',
        parameters=[
            {
                'use_sim_time': use_sim_time,
                'frame_prefix': frame_prefix,
                'initial_soc': float(
                    LaunchConfiguration('initial_soc').perform(context)),
                'discharge_rate': float(
                    LaunchConfiguration('discharge_rate').perform(context)),
                'idle_discharge_rate': float(
                    LaunchConfiguration('idle_discharge_rate').perform(context)),
                'charge_rate': float(
                    LaunchConfiguration('charge_rate').perform(context)),
                'quantize_steps': int(
                    LaunchConfiguration('quantize_steps').perform(context)),
                'chargers': LaunchConfiguration('chargers').perform(context),
                'charge_after_idle_sec': float(
                    LaunchConfiguration('charge_after_idle_sec').perform(
                        context)),
            }
        ],
    )

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='bridge_ros_gz',
        namespace=namespace,
        parameters=[
            {
                'config_file': bridge_config_file,
                'use_sim_time': use_sim_time,
            }
        ],
        output='screen',
    )

    spawn_model = Node(
        condition=IfCondition(use_simulator),
        package='ros_gz_sim',
        executable='create',
        namespace=namespace,
        output='screen',
        arguments=[
            '-name', robot_name,
            '-topic', 'robot_description',
            '-x', pose['x'], '-y', pose['y'], '-z', pose['z'],
            '-R', pose['R'], '-P', pose['P'], '-Y', pose['Y']],
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # The pose bridge publishes the model pose as <world> -> <robot_name>.
    # Connect the model frame to the root frame of the robot model. With
    # publish_odom, insert a per-robot identity odom frame in between so the
    # tree is <world> -> <robot_name> -> <ns>/odom -> <ns>/center, mirroring
    # toio_ros2's odom -> center (map -> <ns>/center stays the ground-truth
    # pose; the /odom topic carries the wheel odometry). Doing it here rather
    # than in simulation.launch gives every robot its own odom frame.
    if publish_odom:
        odom_frame_tfs = [
            Node(
                package='tf2_ros',
                executable='static_transform_publisher',
                name='odom_static_transform_publisher',
                namespace=namespace,
                output='screen',
                arguments=['0', '0', '0', '0', '0', '0',
                           robot_name, frame_prefix + 'odom'],
                parameters=[{'use_sim_time': use_sim_time}],
            ),
            Node(
                package='tf2_ros',
                executable='static_transform_publisher',
                name='center_static_transform_publisher',
                namespace=namespace,
                output='screen',
                arguments=['0', '0', '0', '0', '0', '0',
                           frame_prefix + 'odom', frame_prefix + 'center'],
                parameters=[{'use_sim_time': use_sim_time}],
            ),
        ]
    else:
        odom_frame_tfs = [
            Node(
                package='tf2_ros',
                executable='static_transform_publisher',
                name='center_static_transform_publisher',
                namespace=namespace,
                output='screen',
                arguments=['0', '0', '0', '0', '0', '0',
                           robot_name, frame_prefix + 'center'],
                parameters=[{'use_sim_time': use_sim_time}],
            ),
        ]

    return [
        remove_temp_bridge_file,
        start_robot_state_publisher_cmd,
        bridge,
        led_node,
        sound_node,
        motion_node,
        battery_node,
        spawn_model,
    ] + odom_frame_tfs


def generate_launch_description():
    desc_dir = get_package_share_directory('toio_description')

    # Declare the launch arguments
    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Top-level namespace')

    declare_use_simulator_cmd = DeclareLaunchArgument(
        'use_simulator',
        default_value='True',
        description='Whether to start the simulator')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true',
    )

    declare_use_robot_state_pub_cmd = DeclareLaunchArgument(
        'use_robot_state_pub',
        default_value='True',
        description='Whether to start the robot state publisher')

    declare_robot_name_cmd = DeclareLaunchArgument(
        'robot_name',
        default_value='toio',
        description='name of the robot')

    declare_frame_prefix_cmd = DeclareLaunchArgument(
        'frame_prefix',
        default_value='',
        description='Prefix of the TF frames (e.g. "toio1/")')

    declare_robot_sdf_cmd = DeclareLaunchArgument(
        'robot_sdf',
        default_value=os.path.join(desc_dir, 'robot', 'toio.urdf.xacro'),
        description='Full path to robot sdf file to spawn the robot in gazebo')

    declare_led_duration_ms_cmd = DeclareLaunchArgument(
        'led_duration_ms',
        default_value='0',
        description='Lighting time of the indicator LED in milliseconds. '
                    '0 keeps it lit until the next command, 10-2550 turns it '
                    'off once the time has elapsed')

    declare_led_light_intensity_cmd = DeclareLaunchArgument(
        'led_light_intensity',
        default_value='1.0',
        description='Intensity of the light of the indicator lamp while it is '
                    'lit. 0 leaves the lamp lighting up only itself, without '
                    'casting color on the mat around the cube')

    declare_sound_volume_cmd = DeclareLaunchArgument(
        'sound_volume',
        default_value='255',
        description='Volume of the sound effects. Per the toio specification '
                    'this is mute or full volume only: 0 is mute and every '
                    'other value is the maximum volume')

    declare_imu_interval_ms_cmd = DeclareLaunchArgument(
        'imu_interval_ms',
        default_value='100',
        description='Notification interval of the posture angle behind '
                    '/toio/imu, in milliseconds. The IMU sensor update rate is '
                    '1000/imu_interval_ms Hz (default 10Hz)')

    declare_publish_odom_cmd = DeclareLaunchArgument(
        'publish_odom',
        default_value='True',
        description='Publish /odom and the map -> odom -> center TF tree. '
                    'False restores the plain map -> center transform')

    declare_publish_battery_cmd = DeclareLaunchArgument(
        'publish_battery',
        default_value='False',
        description='Publish a simulated /toio/battery_state (SoC that drops '
                    'while running and rises on a charger) so RMF ChargeBattery '
                    'can fire in sim. Off by default: the reported SoC stays 1.0')

    declare_initial_soc_cmd = DeclareLaunchArgument(
        'initial_soc', default_value='1.0',
        description='Initial state of charge (0.0-1.0) for the simulated battery')

    declare_discharge_rate_cmd = DeclareLaunchArgument(
        'discharge_rate', default_value='0.005',
        description='State of charge lost per second while moving (SoC/s)')

    declare_idle_discharge_rate_cmd = DeclareLaunchArgument(
        'idle_discharge_rate', default_value='0.0005',
        description='State of charge lost per second while idle (SoC/s)')

    declare_charge_rate_cmd = DeclareLaunchArgument(
        'charge_rate', default_value='0.05',
        description='State of charge gained per second while charging (SoC/s)')

    declare_quantize_steps_cmd = DeclareLaunchArgument(
        'quantize_steps', default_value='10',
        description="Round the reported SoC to this many steps. Default 10 "
                    "mirrors the real cube's 10 %% increments; set 0 for a "
                    'smooth, continuous value (e.g. for a nicer demo readout)')

    declare_chargers_cmd = DeclareLaunchArgument(
        'chargers', default_value='',
        description='Charger positions in the map frame as "x y radius,...". '
                    'When set, the cube charges only within a charger radius; '
                    'when empty it falls back to charging after '
                    'charge_after_idle_sec of standing still')

    declare_charge_after_idle_sec_cmd = DeclareLaunchArgument(
        'charge_after_idle_sec', default_value='10.0',
        description='Fallback (no chargers set): treat the cube as charging '
                    'after it has stood still this many seconds')

    # Create the launch description and populate
    ld = LaunchDescription()
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_robot_name_cmd)
    ld.add_action(declare_frame_prefix_cmd)
    ld.add_action(declare_robot_sdf_cmd)
    ld.add_action(declare_led_duration_ms_cmd)
    ld.add_action(declare_led_light_intensity_cmd)
    ld.add_action(declare_sound_volume_cmd)
    ld.add_action(declare_imu_interval_ms_cmd)
    ld.add_action(declare_publish_odom_cmd)
    ld.add_action(declare_publish_battery_cmd)
    ld.add_action(declare_initial_soc_cmd)
    ld.add_action(declare_discharge_rate_cmd)
    ld.add_action(declare_idle_discharge_rate_cmd)
    ld.add_action(declare_charge_rate_cmd)
    ld.add_action(declare_quantize_steps_cmd)
    ld.add_action(declare_chargers_cmd)
    ld.add_action(declare_charge_after_idle_sec_cmd)
    ld.add_action(declare_use_simulator_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_use_robot_state_pub_cmd)

    ld.add_action(OpaqueFunction(function=launch_setup))
    return ld
