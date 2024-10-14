from launch import LaunchDescription
from launch_ros.actions import Node

# launch file für die linienerkennung mit weißen linien
def generate_launch_description():
    return LaunchDescription([
        Node(
            package='robotik_projekt',
            executable='line_following',

            # activate output
            output='screen',
            emulate_tty=True,
            arguments=[('__log_level:=debug')],
            remappings=[
               ('/line_following_twist', '/cmd_vel')
            ],
        ),
    ])
