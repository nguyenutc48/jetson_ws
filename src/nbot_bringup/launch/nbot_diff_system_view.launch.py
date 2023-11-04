from launch import LaunchDescription
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Get URDF via xacro
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [
                    FindPackageShare("nbot_description"),
                    "urdf",
                    "nbot_diff_system.urdf.xacro",
                ]
            ),
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    robot_controllers_param = PathJoinSubstitution(
        [
            FindPackageShare("nbot_bringup"),
            "config",
            "nbot_diff_controller.yaml",
        ]
    )

    robot_localization_param = PathJoinSubstitution(
        [
            FindPackageShare("nbot_bringup"),
            "config",
            "ekf.yaml",
        ]
    )

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, robot_controllers_param],
        remappings=[
            # ("/imu_sensor_node/imu", "/_imu/data_raw"),
            # ("~/motors_cmd", "/_motors_cmd"),
            ("/odom", "/wheels/odom"),
            ("/nbot_diff_base_controller/cmd_vel_unstamped", "/cmd_vel"),
        ],
    )

    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[robot_description],
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner.py",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
        ],
    )

    robot_controller_spawner = Node(
        package="controller_manager",
        executable="spawner.py",
        arguments=[
            "nbot_diff_base_controller",
            "--controller-manager",
            "/controller_manager",
        ],
    )

    # Delay start of robot_controller after joint_state_broadcaster
    delay_robot_controller_spawner_after_joint_state_broadcaster_spawner = (
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=joint_state_broadcaster_spawner,
                on_exit=[robot_controller_spawner],
            )
        )
    )

    imu_node = Node(
        package="nbot_mpu9250",
        executable="nbot_mpu9250",
        name="nbot_mpu9250",
            parameters=[
            {"acceleration_scale": [1.0072387165748442, 1.0081436035838134, 0.9932769089604535], 
            "acceleration_bias": [0.17038044467587418, 0.20464685207217453, -0.12461014438322202], 
            "gyro_bias": [0.0069376404996494, -0.0619247665634732, 0.05717760948453845], 
            "magnetometer_bias": [0.4533159894397744, 3.4555818146055564, -5.984038606178013], 
            "magnetometer_transform": [   0.9983016121720226, 0.044890057238382707, 0.007231924972024632, 
                                0.044890057238382707, 1.2981683205953654, -0.1173361838042438, 
                                0.007231924972024633, -0.11733618380424381, 0.7835617468652673]}  
            ],
        remappings=[("/imu","/imu/data")]
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

    # robot_localization_node = Node(
    #     package='robot_localization',
    #     executable='ekf_node',
    #     name='ekf_filter_node',
    #     output='screen',
    #     parameters=[robot_localization_param],
    # ),

    # # Delay start of imu_broadcaster after robot_controller
    # # when spawning without delay ros2_control_node sometimes crashed
    # delay_robot_localization_after_robot_controller_spawner = RegisterEventHandler(
    #     event_handler=OnProcessExit(
    #         target_action=robot_localization_node,
    #         on_exit=[robot_controller_spawner],
    #     )
    # )

    actions = [
        control_node,
        robot_state_pub_node,
        joint_state_broadcaster_spawner,
        delay_robot_controller_spawner_after_joint_state_broadcaster_spawner,
        imu_node,
        scan_node,
        # delay_robot_localization_after_robot_controller_spawner,
        # robot_localization_node
    ]

    return LaunchDescription(actions)