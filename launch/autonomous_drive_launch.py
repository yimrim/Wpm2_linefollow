from launch_ros.actions import Node

from launch import LaunchDescription


def generate_launch_description():
    return LaunchDescription([
        # Linienverfolgungs-Node
        Node(
            package='robotik_projekt',
            executable='line_following',
            output='screen',
            emulate_tty=True,
            arguments=['__log_level:=debug']
        ),
        # Ampelerkennung
        Node(
            package='robotik_projekt',
            executable='stoplight',
            output='screen',
            emulate_tty=True,
            arguments=['__log_level:=debug']
        ),
        # Hindernisvermeidungs-Node mit Laserscanner
        # Node(
        #     package='robotik_projekt',
        #     executable='drive_with_laserscanner',
        #     output='screen',
        #     emulate_tty=True,
        #     arguments=['__log_level:=debug'],
        # ),
        # State Machine Kontrollnode
        Node(
            package='robotik_projekt',
            executable='state_machine',
            output='screen',
            emulate_tty=True,
            arguments=['__log_level:=debug'],
        )
    ])
