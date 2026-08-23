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
    AppendEnvironmentVariable,
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnShutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression

from launch_ros.actions import Node


def generate_launch_description():
    # Get the launch directory
    sim_dir = get_package_share_directory('toio_gazebo')
    desc_dir = get_package_share_directory('toio_description')
    launch_dir = os.path.join(sim_dir, 'launch')

    # Create the launch configuration variables
    namespace = LaunchConfiguration('namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')

    # Launch configuration variables specific to simulation
    use_simulator = LaunchConfiguration('use_simulator')
    use_robot_state_pub = LaunchConfiguration('use_robot_state_pub')
    headless = LaunchConfiguration('headless')
    world = LaunchConfiguration('world')
    world_frame = LaunchConfiguration('world_frame')
    pose = {
        'x': LaunchConfiguration('x_pose', default='0.05'),
        'y': LaunchConfiguration('y_pose', default='-0.05'),
        'z': LaunchConfiguration('z_pose', default='0.01'),
        'R': LaunchConfiguration('roll', default='0.00'),
        'P': LaunchConfiguration('pitch', default='0.00'),
        'Y': LaunchConfiguration('yaw', default='0.00'),
    }
    robot_name = LaunchConfiguration('robot_name')
    robot_sdf = LaunchConfiguration('robot_sdf')
    frame_prefix = LaunchConfiguration('frame_prefix')
    imu_interval_ms = LaunchConfiguration('imu_interval_ms')
    publish_odom = LaunchConfiguration('publish_odom')
    led_duration_ms = LaunchConfiguration('led_duration_ms')
    led_light_intensity = LaunchConfiguration('led_light_intensity')
    sound_volume = LaunchConfiguration('sound_volume')
    publish_battery = LaunchConfiguration('publish_battery')
    initial_soc = LaunchConfiguration('initial_soc')
    discharge_rate = LaunchConfiguration('discharge_rate')
    idle_discharge_rate = LaunchConfiguration('idle_discharge_rate')
    charge_rate = LaunchConfiguration('charge_rate')
    quantize_steps = LaunchConfiguration('quantize_steps')
    chargers = LaunchConfiguration('chargers')
    charge_after_idle_sec = LaunchConfiguration('charge_after_idle_sec')

    # Declare the launch arguments
    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace', default_value='', description='Top-level namespace'
    )

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='True',
        description='Use simulation (Gazebo) clock if true',
    )

    declare_use_simulator_cmd = DeclareLaunchArgument(
        'use_simulator',
        default_value='True',
        description='Whether to start the simulator',
    )

    declare_use_robot_state_pub_cmd = DeclareLaunchArgument(
        'use_robot_state_pub',
        default_value='True',
        description='Whether to start the robot state publisher',
    )

    declare_simulator_cmd = DeclareLaunchArgument(
        'headless', default_value='False', description='Whether to execute gzclient)'
    )

    declare_world_cmd = DeclareLaunchArgument(
        'world',
        default_value=os.path.join(sim_dir, 'worlds', 'toio_a4_map.sdf'),
        description='Full path to world model file to load',
    )

    declare_world_frame_cmd = DeclareLaunchArgument(
        'world_frame',
        default_value='toio_a4_map',
        description='frame_id of world',
    )

    declare_robot_name_cmd = DeclareLaunchArgument(
        'robot_name', default_value='toio', description='name of the robot'
    )

    declare_robot_sdf_cmd = DeclareLaunchArgument(
        'robot_sdf',
        default_value=os.path.join(desc_dir, 'robot', 'toio.urdf.xacro'),
        description='Full path to robot sdf file to spawn the robot in gazebo',
    )

    declare_frame_prefix_cmd = DeclareLaunchArgument(
        'frame_prefix',
        default_value='',
        description='Prefix of the TF frames (e.g. "toio1/")',
    )

    declare_led_duration_ms_cmd = DeclareLaunchArgument(
        'led_duration_ms',
        default_value='0',
        description='Lighting time of the indicator LED in milliseconds. '
                    '0 keeps it lit until the next command, 10-2550 turns it '
                    'off once the time has elapsed',
    )

    declare_led_light_intensity_cmd = DeclareLaunchArgument(
        'led_light_intensity',
        default_value='1.0',
        description='Intensity of the light of the indicator lamp while '
                    'it is lit. 0 leaves the lamp lighting up only '
                    'itself, without casting color on the mat around the '
                    'cube',
    )

    declare_sound_volume_cmd = DeclareLaunchArgument(
        'sound_volume',
        default_value='255',
        description='Volume of the sound effects. Per the toio specification '
                    'this is mute or full volume only: 0 is mute and every '
                    'other value is the maximum volume',
    )

    declare_imu_interval_ms_cmd = DeclareLaunchArgument(
        'imu_interval_ms',
        default_value='100',
        description='Notification interval of the posture angle behind '
                    '/toio/imu, in milliseconds (IMU update rate is '
                    '1000/imu_interval_ms Hz, default 10Hz)',
    )

    declare_publish_odom_cmd = DeclareLaunchArgument(
        'publish_odom',
        default_value='True',
        description='Publish /odom and the map -> odom -> center TF tree. '
                    'False restores the plain map -> center transform',
    )

    declare_publish_battery_cmd = DeclareLaunchArgument(
        'publish_battery', default_value='False',
        description='Publish a simulated /toio/battery_state so RMF '
                    'ChargeBattery can fire in sim (off by default)')
    declare_initial_soc_cmd = DeclareLaunchArgument(
        'initial_soc', default_value='1.0')
    declare_discharge_rate_cmd = DeclareLaunchArgument(
        'discharge_rate', default_value='0.01')
    declare_idle_discharge_rate_cmd = DeclareLaunchArgument(
        'idle_discharge_rate', default_value='0.001')
    declare_charge_rate_cmd = DeclareLaunchArgument(
        'charge_rate', default_value='0.05')
    declare_quantize_steps_cmd = DeclareLaunchArgument(
        'quantize_steps', default_value='0')
    declare_chargers_cmd = DeclareLaunchArgument(
        'chargers', default_value='',
        description='Charger positions "x y radius,..." in the map frame; '
                    'empty falls back to charging when idle')
    declare_charge_after_idle_sec_cmd = DeclareLaunchArgument(
        'charge_after_idle_sec', default_value='10.0')

    # The clock is shared by all robots, so it is bridged here instead of
    # spawn_toio.launch.py to avoid duplicated bridges in multi-robot setup.
    clock_bridge = Node(
        condition=IfCondition(use_simulator),
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='clock_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        output='screen',
    )

    # map -> world frame is a single shared, identity transform for the whole
    # Gazebo world. The per-robot 'odom' frame (map -> ... -> <ns>/odom ->
    # <ns>/center) is inserted in spawn_toio.launch.py so that every robot,
    # including the ones spawned into a running Gazebo, gets its own odom
    # frame instead of hanging under the first robot's.
    map_static_transform_publisher_cmd = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='map_static_transform_publisher',
        output='screen',
        arguments=['0', '0', '0', '0', '0', '0', 'map', world_frame],
    )

    # The SDF file for the world is a xacro file because we wanted to
    # conditionally load the SceneBroadcaster plugin based on wheter we're
    # running in headless mode. But currently, the Gazebo command line doesn't
    # take SDF strings for worlds, so the output of xacro needs to be saved into
    # a temporary file and passed to Gazebo.
    world_sdf = tempfile.mktemp(prefix='nav2_', suffix='.sdf')
    world_sdf_xacro = ExecuteProcess(
        cmd=['xacro', '-o', world_sdf, ['headless:=', headless], world])
    gazebo_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ros_gz_sim'), 'launch',
                         'gz_sim.launch.py')),
        launch_arguments={'gz_args': ['-r -s ', world_sdf]}.items(),
        condition=IfCondition(use_simulator))

    remove_temp_sdf_file = RegisterEventHandler(event_handler=OnShutdown(
        on_shutdown=[
            OpaqueFunction(function=lambda _: os.remove(world_sdf))
        ]))

    set_env_vars_resources = AppendEnvironmentVariable(
            'GZ_SIM_RESOURCE_PATH',
            os.path.join(sim_dir, 'worlds'))
    gazebo_client = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ros_gz_sim'),
                         'launch',
                         'gz_sim.launch.py')
        ),
        condition=IfCondition(PythonExpression([use_simulator, ' and not ', headless])),
        launch_arguments={'gz_args': ['-v4 -g ']}.items(),
    )

    gz_robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_dir, 'spawn_toio.launch.py')),
        launch_arguments={'namespace': namespace,
                          'use_simulator': use_simulator,
                          'use_sim_time': use_sim_time,
                          'use_robot_state_pub': use_robot_state_pub,
                          'robot_name': robot_name,
                          'frame_prefix': frame_prefix,
                          'robot_sdf': robot_sdf,
                          'led_duration_ms': led_duration_ms,
                          'led_light_intensity': led_light_intensity,
                          'sound_volume': sound_volume,
                          'imu_interval_ms': imu_interval_ms,
                          'publish_odom': publish_odom,
                          'publish_battery': publish_battery,
                          'initial_soc': initial_soc,
                          'discharge_rate': discharge_rate,
                          'idle_discharge_rate': idle_discharge_rate,
                          'charge_rate': charge_rate,
                          'quantize_steps': quantize_steps,
                          'chargers': chargers,
                          'charge_after_idle_sec': charge_after_idle_sec,
                          'x_pose': pose['x'],
                          'y_pose': pose['y'],
                          'z_pose': pose['z'],
                          'roll': pose['R'],
                          'pitch': pose['P'],
                          'yaw': pose['Y']}.items())

    # Create the launch description and populate
    ld = LaunchDescription()

    # Declare the launch options
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_use_simulator_cmd)
    ld.add_action(declare_use_robot_state_pub_cmd)
    ld.add_action(declare_simulator_cmd)
    ld.add_action(declare_world_cmd)
    ld.add_action(declare_world_frame_cmd)
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

    ld.add_action(set_env_vars_resources)
    ld.add_action(world_sdf_xacro)
    ld.add_action(remove_temp_sdf_file)
    ld.add_action(gz_robot)
    ld.add_action(gazebo_server)
    ld.add_action(gazebo_client)

    ld.add_action(clock_bridge)
    ld.add_action(map_static_transform_publisher_cmd)

    return ld
