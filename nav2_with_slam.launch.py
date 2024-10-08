from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='slam_toolbox',
            executable='online_async_launch.py',
            name='slam_toolbox',
            output='screen',
            parameters=['src/slam_toolbox.yaml'],

        ),
        Node(
            package='nav2_bringup',
            executable='bringup_launch.py',
            name='nav2',
            output='screen',
            parameters=['src/nav2_params.yaml'],
            arguments=['--use-sim-time', 'false']
        )
    ])
