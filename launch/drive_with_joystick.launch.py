from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package = 'joy',
            executable = 'joy_node',
            name = 'joystick'
            
        ),
        Node(
            package = 'my_onboarding_project2',
            executable = 'joystick_interpreter',
            name = 'joystick_interpreter'

        ),
        Node(
            package = 'my_onboarding_project2',
            executable= 'canbus_node',
            name = 'canbus_node'

        )
    ])