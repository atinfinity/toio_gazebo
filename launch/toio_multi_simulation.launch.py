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

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='True',
        description='Use simulation (Gazebo) clock if true',
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
                          'y_pose': '-0.05'}.items())

    # Spawn the second robot into the running Gazebo
    spawn_toio2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_dir, 'spawn_toio.launch.py')),
        launch_arguments={'namespace': 'toio2',
                          'robot_name': 'toio2',
                          'frame_prefix': 'toio2/',
                          'use_sim_time': use_sim_time,
                          'x_pose': '0.24',
                          'y_pose': '-0.14'}.items())

    ld = LaunchDescription()
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_simulator_cmd)
    ld.add_action(declare_world_cmd)
    ld.add_action(declare_world_frame_cmd)

    ld.add_action(simulation_toio1)
    ld.add_action(spawn_toio2)
    return ld
