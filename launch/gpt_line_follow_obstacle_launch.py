from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Linienverfolgungs-Node
        Node(
            package='robotik_projekt',
            executable='line_following',
            output='screen',
            emulate_tty=True,
            arguments=['__log_level:=debug'],
            parameters=[
                {'boundary_left': 90},   # Optimierung der Grenzen
                {'boundary_right': 200}, # Erweitert für mehr Spielraum
                {'threshold_line': 110}, # Angepasster Schwellenwert für präzisere Erkennung
                {'line_follow_speed': 0.3} # Dynamische Geschwindigkeitsanpassung
            ],
            remappings=[
                ('/cmd_vel', '/line_follower/cmd_vel')  # Bessere Steuerung durch eigenes Topic
            ]
        ),

        # Hindernisvermeidungs-Node mit Laserscanner
        Node(
            package='robotik_projekt',
            executable='drive_with_laserscanner',
            output='screen',
            emulate_tty=True,
            arguments=['__log_level:=debug'],
            parameters=[
                {'min_obstacle_distance': 0.4},  # Kleinere Distanz für engere Hindernisse
                {'scan_angle': 120},  # Größerer Winkel für breitere Abdeckung
                {'avoidance_speed': 0.2},  # Geringere Geschwindigkeit bei Hindernisnähe
                {'max_obstacle_avoid_time': 3.0}  # Max Zeit, um ein Hindernis zu umfahren
            ],
            remappings=[
                ('/cmd_vel', '/obstacle_avoider/cmd_vel'),  # Trennung von Bewegungs-Topics
                ('/laser_scan', '/scan_filtered')  # Nur gefilterte Scans verwenden
            ]
        ),

        # Neue Node für die Integration von Linienverfolgung und Hindernisvermeidung
        Node(
            package='robotik_projekt',
            executable='integrator',
            output='screen',
            emulate_tty=True,
            arguments=['__log_level:=debug'],
            parameters=[
                {'reaction_threshold': 0.5},  # Schwelle, wann auf Hindernisse reagiert wird
                {'priority': 'obstacle_avoidance'}  # Hindernisvermeidung hat Vorrang
            ],
            remappings=[
                ('/line_follower/cmd_vel', '/cmd_vel'),  # Ausgabe von Linienverfolgung
                ('/obstacle_avoider/cmd_vel', '/cmd_vel')  # Ausgabe von Hindernisvermeidung
            ]
        )
    ])
