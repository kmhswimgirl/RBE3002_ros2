import os
from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():   

    turtlebot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('turtlebot3_gazebo'), 'launch'), '/empty_world.launch.py']),
        launch_arguments = [
            ('use_sim_time', 'True')
        ]
    )

    rviz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('turtlebot3_bringup'), 'launch'), '/rviz2.launch.py']),
            launch_arguments = [
            ('use_sim_time', 'True')
        ]
    )

    lab2_node = Node(
        package='control',
        executable='lab2_topic_only.py',
        name='lab2_driver',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    static_transform = Node(package = "tf2_ros", 
                       executable = "static_transform_publisher",
                       arguments=["0.5", "0.2", "0", "0.78", "0", "0", "odom", "map"] 
            )


    return LaunchDescription([
        turtlebot,
        rviz,
        lab2_node,
        static_transform
    ])


