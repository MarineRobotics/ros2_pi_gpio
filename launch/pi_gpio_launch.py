from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='pi_gpio',
            executable='builtin_gpio_server',
            name='builtin_gpio_server',
            output='screen',
        ),
        Node(
            package='pi_gpio',
            executable='pcf8574_gpio_server',
            name='pcf8574_gpio_server',
            output='screen',
        ),
    ])
