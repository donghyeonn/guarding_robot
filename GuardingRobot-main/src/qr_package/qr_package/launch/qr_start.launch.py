from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='qr_package',
            executable='qr_detector',
            name='qr_detector',
            output='screen'
        ),
        Node(
            package='qr_package',
            executable='qr_database',
            name='qr_database',
            output='screen'
        ),
        Node(
            package='qr_package',
            executable='qr_speak',
            name='qr_speak',
            output='screen'
        ),
    ])
