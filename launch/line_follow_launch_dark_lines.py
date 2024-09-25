from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='robotik_projekt',
            executable='line_following_dark_lines',

            # activate output
            output='screen',
            emulate_tty=True,
            arguments=[('__log_level:=debug')],
            remappings=[
               ('/line_following_twist', '/cmd_vel')
            ],
            # parameters=[
            #     {'boundary_left': 92},
            #     {'boundary_right': 198},
            #     {'threshold_line': 102}
            # ]
        ),
    ])
