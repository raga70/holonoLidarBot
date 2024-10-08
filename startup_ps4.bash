#!/bin/bash
source /opt/ros/humble/setup.bash

# Run the first ROS package
ros2 launch slam_toolbox slam_launch.py &

# Run the second ROS package
rviz2 -d src/rviz_config.rviz &

# Run the Python program
python3 src/publisher_controller_ps4.py &

# Wait for all background processes to finish
wait

echo "All processes have completed."