#!/bin/bash

# Source the ROS2 and workspace setup
source /home/ubuntu/ros2_ws/install/setup.bash
source /opt/ros/humble/setup.bash

# Start the first process in a detached screen session
screen -dmS wheel_kinematics_odom bash -c "source /opt/ros/humble/setup.bash &&  python3 /home/ubuntu/holonoLidarBot/my_package/my_package/odom_kin_processing_node.py"

# Start the second process in another detached screen session
screen -dmS rplidar_launch bash -c "source /home/ubuntu/ros2_ws/install/setup.bash && source /home/ubuntu/ros2_ws/install/setup.bash && source /opt/ros/humble/setup.bash && ros2 launch /home/ubuntu/ros2_ws/src/install/rplidar_ros/share/rplidar_ros/launch/view_rplidar_a2m8_launch.py"

# Start the static transform publisher in another detached screen session
screen -dmS static_transform bash -c "source /home/ubuntu/ros2_ws/install/setup.bash && source /opt/ros/humble/setup.bash && ros2 run tf2_ros static_transform_publisher 0.0 0 0 -1.5708 0 0 base_link laser"

echo "Processes started in separate screen sessions:"
echo "  - wheel_kinematics (screen session: wheel_kinematics_odom)"
echo "  - rplidar_launch (screen session: rplidar_launch)"
echo "  - static_transform_publisher (screen session: static_transform)"
