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

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    sim_dir = get_package_share_directory('toio_gazebo')
    launch_dir = os.path.join(sim_dir, 'launch')

    use_sim_time = LaunchConfiguration('use_sim_time')
    headless = LaunchConfiguration('headless')
    world = LaunchConfiguration('world')
    world_frame = LaunchConfiguration('world_frame')
    publish_battery = LaunchConfiguration('publish_battery')
    chargers = LaunchConfiguration('chargers')
    discharge_rate = LaunchConfiguration('discharge_rate')
    idle_discharge_rate = LaunchConfiguration('idle_discharge_rate')
    charge_rate = LaunchConfiguration('charge_rate')
    quantize_steps = LaunchConfiguration('quantize_steps')

    # Battery is opt-in (off by default) and, when on, both cubes share the
    # same charger list and rates. See spawn_toio.launch.py for the semantics.
    battery_args = {
        'publish_battery': publish_battery,
        'chargers': chargers,
        'discharge_rate': discharge_rate,
        'idle_discharge_rate': idle_discharge_rate,
        'charge_rate': charge_rate,
        'quantize_steps': quantize_steps,
    }

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='True',
        description='Use simulation (Gazebo) clock if true',
    )

    declare_publish_battery_cmd = DeclareLaunchArgument(
        'publish_battery', default_value='False',
        description='Publish a simulated /toioN/toio/battery_state for both '
                    'cubes so RMF ChargeBattery can fire in sim (off by default)')
    declare_chargers_cmd = DeclareLaunchArgument(
        'chargers', default_value='',
        description='Charger positions "x y radius,..." in the map frame; '
                    'empty falls back to charging when idle')
    declare_discharge_rate_cmd = DeclareLaunchArgument(
        'discharge_rate', default_value='0.005')
    declare_idle_discharge_rate_cmd = DeclareLaunchArgument(
        'idle_discharge_rate', default_value='0.0005')
    declare_charge_rate_cmd = DeclareLaunchArgument(
        'charge_rate', default_value='0.05')
    declare_quantize_steps_cmd = DeclareLaunchArgument(
        'quantize_steps', default_value='10')

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

    # Launch Gazebo and spawn the first robot
    simulation_toio1 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_dir, 'simulation.launch.py')),
        launch_arguments={'namespace': 'toio1',
                          'robot_name': 'toio1',
                          'frame_prefix': 'toio1/',
                          'use_sim_time': use_sim_time,
                          'headless': headless,
                          'world': world,
                          'world_frame': world_frame,
                          'x_pose': '0.05',
                          'y_pose': '-0.05',
                          **battery_args}.items())

    # Spawn the second robot into the running Gazebo
    spawn_toio2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_dir, 'spawn_toio.launch.py')),
        launch_arguments={'namespace': 'toio2',
                          'robot_name': 'toio2',
                          'frame_prefix': 'toio2/',
                          'use_sim_time': use_sim_time,
                          'x_pose': '0.24',
                          'y_pose': '-0.14',
                          **battery_args}.items())

    ld = LaunchDescription()
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_simulator_cmd)
    ld.add_action(declare_world_cmd)
    ld.add_action(declare_world_frame_cmd)
    ld.add_action(declare_publish_battery_cmd)
    ld.add_action(declare_chargers_cmd)
    ld.add_action(declare_discharge_rate_cmd)
    ld.add_action(declare_idle_discharge_rate_cmd)
    ld.add_action(declare_charge_rate_cmd)
    ld.add_action(declare_quantize_steps_cmd)

    ld.add_action(simulation_toio1)
    ld.add_action(spawn_toio2)
    return ld
