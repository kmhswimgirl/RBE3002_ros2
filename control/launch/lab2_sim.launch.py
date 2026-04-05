import os
from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.actions import DeclareLaunchArgument

def generate_launch_description():   

    gz_gui = LaunchConfiguration('gz_gui')
    gz_gui_cmd = DeclareLaunchArgument(
            'gz_gui',
            default_value='false',
            description='determines if the GUI part of gazebo is run or not'
    ) 

    gazebo_launch_file = PythonExpression([
        "'",
        os.path.join(get_package_share_directory('turtlebot3_gazebo'), 'launch', 'empty_world.launch.py'),
        "' if '", gz_gui, "' == \"true\" else '",
        os.path.join(get_package_share_directory('control'), 'launch', 'empty_world_2.launch.py'),
        "'"
    ])

    turtlebot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([gazebo_launch_file]),
        launch_arguments=[
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
        executable='tf_controller.py',
        name='controller',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    return LaunchDescription([
        gz_gui_cmd,
        turtlebot,
        # rviz,
        lab2_node
    ])


