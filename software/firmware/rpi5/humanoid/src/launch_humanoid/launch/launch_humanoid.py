from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='mouth',
            executable='speak',
            name='speaker'
        ),
        Node(
            package='ears',
            executable='listen',
            name='mic'
        ),
        Node(
            package='conversation',
            executable='conversation',
            name='conversation'
        ),
        Node(
            package='brain',
            executable='brain',
            name='brain'
        )
    ])

