#!/bin/bash
source /opt/ros/humble/setup.bash

# Run the first ROS package
ros2 launch slam_toolbox slam_launch.py &

# Run the second ROS package
rviz2 &

# Run the Python program
python3 src/publisher_wasd.py &

# Wait for all background processes to finish
wait

echo "All processes have completed."
