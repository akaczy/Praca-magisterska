#!/usr/bin/env python3
import os
import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
from webots_ros2_driver.utils import controller_url_prefix
from webots_ros2_driver.webots_launcher import WebotsLauncher

def generate_launch_description():
    package_dir = get_package_share_directory('robot_symulacja')
    rosbot_description_pkg = get_package_share_directory('rosbot_description')
    xacro_path = os.path.join(rosbot_description_pkg, 'urdf', 'rosbot.urdf.xacro')

    robot_description = ParameterValue(
        Command(['xacro ', xacro_path, ' use_sim:=true simulation_engine:=webots']),
        value_type=str
    )

    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description},
                    {'use_sim_time': True}]
    )

    # Swiat webots
    world = WebotsLauncher(
        world=os.path.join(package_dir, 'worlds', 'rosbot.wbt'),
        ros2_supervisor=True
    )

    # ros2_control parametry
    ros2_ctrl_params = os.path.join(package_dir, 'resource', 'rosbot_controllers.yaml')

    # Webots driver
    driver = Node(
        package='webots_ros2_driver',
        executable='driver',
        additional_env={'WEBOTS_CONTROLLER_URL': controller_url_prefix() + 'rosbot'},
        output='screen',
        parameters=[
            {'robot_description': robot_description},
            {'set_robot_state_publisher': False},
            {'use_sim_time': True},
            ros2_ctrl_params,
        ],
        remappings=[
            ('rosbot_base_controller/cmd_vel_unstamped', 'cmd_vel'),
            ('odom', 'rosbot_base_controller/odom'),
            ('rosbot/laser', '/scan'),
            ('rosbot/rl_range', '/range/rl'),
            ('rosbot/rr_range', '/range/rr'),
            ('rosbot/fl_range', '/range/fl'),
            ('rosbot/fr_range', '/range/fr')
        ]
    )

    # Kontrolery
    jsb = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'joint_state_broadcaster',
            '--controller-manager', '/controller_manager',
            '--controller-manager-timeout', '120',
        ],
    )
    base_ctrl = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'rosbot_base_controller',
            '--controller-manager', '/controller_manager',
            '--controller-manager-timeout', '120',
        ],
    )
    delay_base_ctrl = RegisterEventHandler(
        OnProcessExit(target_action=jsb, on_exit=[base_ctrl])
    )

    ekf_config = os.path.join(package_dir, 'resource', 'ekf.yaml')
    ekf = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_config, {'use_sim_time': True},
                    {'odom0': '/rosbot_base_controller/odom'}]
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='True'),
        world,
        world._supervisor,
        driver,
        RegisterEventHandler(
            OnProcessExit(
                target_action=world._supervisor,
                on_exit=[launch.actions.EmitEvent(event=launch.events.Shutdown())],
            )
        ),
        jsb,
        delay_base_ctrl,
        rsp,
        ekf,
    ])

