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


def launch_setup(context, *args, **kwargs):
    namespace = LaunchConfiguration('namespace').perform(context)
    robot_name = LaunchConfiguration('robot_name').perform(context)
    frame_prefix = LaunchConfiguration('frame_prefix').perform(context)
    robot_sdf = LaunchConfiguration('robot_sdf').perform(context)
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
             'robot_description': Command(
                 ['xacro', ' ', robot_sdf, ' ', 'robot_name:=', robot_name])}
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

    # The pose bridge publishes the model pose as <world> -> <robot_name>,
    # so connect the model frame to the root frame of the robot model.
    center_static_transform_publisher_cmd = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='center_static_transform_publisher',
        namespace=namespace,
        output='screen',
        arguments=['0', '0', '0', '0', '0', '0',
                   robot_name, frame_prefix + 'center'],
        parameters=[{'use_sim_time': use_sim_time}],
    )

    return [
        remove_temp_bridge_file,
        start_robot_state_publisher_cmd,
        bridge,
        spawn_model,
        center_static_transform_publisher_cmd,
    ]


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

    # Create the launch description and populate
    ld = LaunchDescription()
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_robot_name_cmd)
    ld.add_action(declare_frame_prefix_cmd)
    ld.add_action(declare_robot_sdf_cmd)
    ld.add_action(declare_use_simulator_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_use_robot_state_pub_cmd)

    ld.add_action(OpaqueFunction(function=launch_setup))
    return ld
