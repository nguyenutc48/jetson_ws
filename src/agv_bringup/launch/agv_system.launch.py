# Copyright 2020 ros2_control Development Team
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

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    # arg_show_rviz = DeclareLaunchArgument(
    #     "start_rviz",
    #     default_value="false",
    #     description="start RViz automatically with the launch file",
    # )

    # Get URDF via xacro
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare("agv_description"), "urdf", "agv_system.urdf.xacro"]
            ),
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    diffbot_diff_drive_controller = PathJoinSubstitution(
        [
            FindPackageShare("agv_bringup"),
            "config",
            "agv_controller.yaml",
        ]
    )

    node_robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[robot_description],
    )

    controller_manager_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, diffbot_diff_drive_controller],
        output={
            "stdout": "screen",
            "stderr": "screen",
        },
        remappings=[
            # ("/imu_sensor_node/imu", "/_imu/data_raw"),
            # ("~/motors_cmd", "/_motors_cmd"),
            # ("~/motors_response", "/_motors_response"),
            ("/agv_base_controller/cmd_vel_unstamped", "/cmd_vel"),
        ],
    )

    spawn_imu_controller = Node(
        package="controller_manager",
        executable="spawner.py",
        arguments=[
            "imu_broadcaster",
            "--controller-manager",
            "/controller_manager",
            ],
        output="screen",
    )

    spawn_dd_controller = Node(
        package="controller_manager",
        executable="spawner.py",
        arguments=[
            "agv_base_controller",
            "--controller-manager",
            "/controller_manager",
            ],
        output="screen",
    )

    spawn_jsb_controller = Node(
        package="controller_manager",
        executable="spawner.py",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
            ],
        output="screen",
    )

    scan_node =  Node(
        package='rplidar_ros',
        executable='rplidar_composition',
        output='screen',
        parameters=[{
            'serial_port': '/dev/ttyUSB0',
            'frame_id': 'laser_link',
            'angle_compensate': True,
            'scan_mode': 'Standard'
        }]
    )

    return LaunchDescription(
        [
            node_robot_state_publisher,
            controller_manager_node,
            spawn_imu_controller,
            spawn_dd_controller,
            spawn_jsb_controller,
            scan_node,
        ]
    )
