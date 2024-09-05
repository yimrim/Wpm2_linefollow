from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='robotik_projekt',
            executable='stoplight',

            # activate output
            output='screen',
            emulate_tty=True,
            arguments=[('__log_level:=debug')],
        ),
    ])
