from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        # Declare necessary launch arguments
        DeclareLaunchArgument(
            'use_sim_time', default_value='false', description='Use simulation (Gazebo) clock if true'
        ),
        DeclareLaunchArgument(
            'autostart', default_value='true', description='Automatically start nav2 lifecycle nodes'
        ),
        
        # Launch Nav2 with custom parameters
        Node(
            package='nav2_bringup',
            executable='bringup_launch.py',
            output='screen',
            parameters=[
                {'use_sim_time': LaunchConfiguration('use_sim_time')},
                {'autostart': LaunchConfiguration('autostart')},
                
                # Set robot footprint to 35cm x 35cm (0.35m x 0.35m)
                {'robot_base_frame': 'base_footprint'},
                {'footprint': [[0.175, 0.175], [-0.175, 0.175], [-0.175, -0.175], [0.175, -0.175]]},  # Square footprint
                
                # Set velocity limits for holonomic motion
                {'min_vel_x': -1.0},   # Forward/backward linear velocity
                {'max_vel_x': 1.0},
                {'min_vel_y': -1.0},   # Lateral (sideways) velocity for holonomic movement
                {'max_vel_y': 1.0},
                {'min_vel_theta': -1.0},  # Angular velocity
                {'max_vel_theta': 1.0}
            ],
        ),
    ])

if __name__ == '__main__':
    generate_launch_description()
