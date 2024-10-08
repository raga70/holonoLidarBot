#!/bin/bash

# Source the ROS2 and workspace setup
source /home/ubuntu/ros2_ws/install/setup.bash
source /opt/ros/humble/setup.bash

# Start the first process in a detached screen session
screen -dmS wheel_kinematics_odom bash -c "source /opt/ros/humble/setup.bash &&  python3 /home/ubuntu/testMainBranch/holonoLidarBot/src/odom_kin_processing_node.py"

# Start the second process in another detached screen session
screen -dmS rplidar_launch bash -c "sudo usermod -aG dialout $USER && source /home/ubuntu/ros2_ws/install/setup.bash && source /home/ubuntu/ros2_ws/install/setup.bash && source /opt/ros/humble/setup.bash && ros2 launch /home/ubuntu/ros2_ws/src/install/rplidar_ros/share/rplidar_ros/launch/rplidar_a2m8_launch.py"

# Start the static transform publisher in another detached screen session
screen -dmS static_transform bash -c "source /home/ubuntu/ros2_ws/install/setup.bash && source /opt/ros/humble/setup.bash && ros2 run tf2_ros static_transform_publisher 0.0 0 0 0 0 0 base_footprint base_link"
screen -dmS static_transform bash -c "source /home/ubuntu/ros2_ws/install/setup.bash && source /opt/ros/humble/setup.bash && ros2 run tf2_ros static_transform_publisher 0.0 0 0 3.1416 0 0 base_link laser"


# Launch SLAM Toolbox
ros2 run slam_toolbox async_slam_toolbox_node --ros-args --param use_sim_time:=false --param map_topic:=/map --param base_frame:=base_link --param odom_frame:=odom --param scan_topic:=/scan --param minimum_travel_distance:=0.1 &

# Launch Nav2
ros2 launch nav2_bringup online_async_launch.py params_file:=src/nav2_params.yaml &


echo "Processes started in separate screen sessions:"
echo "  - wheel_kinematics (screen session: wheel_kinematics_odom)"
echo "  - rplidar_launch (screen session: rplidar_launch)"
echo "  - static_transform_publisher (screen session: static_transform)"