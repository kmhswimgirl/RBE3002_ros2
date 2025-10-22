#!/usr/bin/env python3

from launch.actions import DeclareLaunchArgument
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

    ## ============= Launch Args ============== ##
    joy_dev_arg = DeclareLaunchArgument('joy_dev', default_value='0')
    stamped_twist_arg = DeclareLaunchArgument('publish_stamped_twist', default_value='true')
    controller_name_arg = DeclareLaunchArgument('controller_name', default_value='xbox_controller.yaml')

    ## ============= Launch Configs ============== ##
    joy_dev = LaunchConfiguration('joy_dev')
    publish_stamped_twist = LaunchConfiguration('publish_stamped_twist')
    controller_name = LaunchConfiguration('controller_name')

    ## ============= Config ============== ##
    config_path = PathJoinSubstitution([
        FindPackageShare('robot_gazebo'),
        'config',
        controller_name]
    )

    ## ============= Joy + Teleop Nodes ============== ##
    joy_node = Node(
        package='joy', 
        executable='joy_node', 
        name='joy_node',
        parameters=[config_path,
            {'device_id': joy_dev}, 
            {'deadzone': 0.3},
            {'autorepeat_rate': 20.0}]
    )

    teleop_node = Node(
        package='teleop_twist_joy', 
        executable='teleop_node',
        name='teleop_twist_joy_node',
        parameters=[config_path, 
            {'publish_stamped_twist': publish_stamped_twist}, 
            {'require_enable_button': False}]
    )

    return LaunchDescription([
        joy_dev_arg,
        stamped_twist_arg,
        controller_name_arg,
        joy_node,
        teleop_node
    ])