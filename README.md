# RBE3002_ros2
Migrating labs 2 &amp; 3 to ROS2 Jazzy.

## Setup Notes:
__turtlebot3_gazebo:__
1. `cd turtlebot3_simulations/turtlebot3_gazebo/params`
2. `nano turtlebot3_burger_bridge.yaml`
3. find line 32 and change `ros_type_name` to be `"geometry_msgs/msg/Twist"`
4. might want to change the same parameter in the corresponding "cam" yaml file

__turtlebot3_navigation2:__
1. `cd turtlebot3/turtlebot3_navigation2/param`
2. `nano burger.yaml`
3. find line 317 and change `enable_stamped_cmd_vel` to be `false`

__turtlebot3_bringup:__
1. `cd turtlebot3/turtlebot3_bringup/param`
2. `nano burger.yaml`
3. change variable `enable_stampled_cmd_vel` to `false`

## Lab 2: Kinematics and Odometry

Base version of lab 2:
`ros2 launch lab2 lab2_t.launch.py`

ex. credit version coming soon!
