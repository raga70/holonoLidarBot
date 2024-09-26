#!/bin/bash
source /opt/ros/humble/setup.bash

echo "Running debug programs."

# Run the first ROS package
rqt_graph &

# Run the second ROS package
ros2 run rqt_tf_tree rqt_tf_tree --force-discover &

# Wait for all background processes to finish
wait

echo "Good luck for debugging!"
