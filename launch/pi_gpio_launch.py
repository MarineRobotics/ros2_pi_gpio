from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='pi_gpio',
            executable='pi_gpio_server',
            name='pi_gpio_server'
        ),
    ])
