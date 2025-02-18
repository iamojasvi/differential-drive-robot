from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='differential-drive-robot',
            executable='differential_drive_controller',
            name='diff_drive_controller',
            output='screen'
        ),
        Node(
            package='differential-drive-robot',
            executable='waypoint_navigation.py',
            name='waypoint_navigator',
            output='screen',
            parameters=[
                {'waypoint_1_x': 2.0, 'waypoint_1_y': 1.0, 'waypoint_2_x': 4.0, 'waypoint_2_y': 3.0}
            ]
        )
    ])
