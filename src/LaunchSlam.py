from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='slam_toolbox',
            executable='sync_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[{
                'use_sim_time': False,
                'slam_toolbox': 'true',
                'odom_frame': 'odom',
                'map_frame': 'map',
                'base_frame': 'base_link',  # Your base frame
                'scan_topic': '/scan',  # Your scan topic
                'update_frame': 'laser',  # Your laser frame
                'resolution': 0.05,  # Grid resolution
            }],
        ),
    ])
