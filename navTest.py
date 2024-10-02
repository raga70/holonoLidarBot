from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        
        
       DeclareLaunchArgument(
            'slam',
            default_value='true',
            description='Whether to run SLAM or just localization'),

        # Nav2 bringup node
        Node(
            package='nav2_bringup',
            executable='bringup_launch.py',
            output='screen',
            parameters=[{
                'use_sim_time': 'false',
                'yaml_filename': 'path_to_your_map.yaml',
                'odom_frame': 'odom',
                'base_frame': 'base_link',
                'update_frame': 'laser',
                'robot_base_frame': 'base_link',
                'footprint': [
                    [-0.175, -0.175],  # back left corner
                    [0.175, -0.175],   # front left corner
                    [0.175, 0.175],    # front right corner
                    [-0.175, 0.175]    # back right corner
                ],
            }],
            remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static')]
        ),


        # Nav2 controller
        Node(
            package='nav2_controller',
            executable='controller_server',
            name='controller_server',
            output='screen',
            parameters=[{
                'use_sim_time': 'false',
                'controller_frequency': 20.0,
                'goal_checker': {'type': 'goal_checker'}
            }]
        ),

        # Nav2 planner
        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            output='screen',
            parameters=[{
                'use_sim_time': 'false',
                'planner_frequency': 5.0,
            }]
        ),

        # Nav2 recoveries
        Node(
            package='nav2_recoveries',
            executable='recoveries_server',
            name='recoveries_server',
            output='screen',
            parameters=[{
                'use_sim_time': 'false',
            }]
        ),

        # Nav2 BT Navigator
        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            output='screen',
            parameters=[{
                'use_sim_time': 'false',
                'bt_tree_xml': LaunchConfiguration('default_bt_tree')
            }]
        )
    ])

