import rclpy
from pi_gpio.builtin_gpio_server import BuiltinGPIOActionServer
from ament_index_python.packages import get_package_share_directory
import os

def main(args=None):
    rclpy.init(args=args)

    package_share_dir = get_package_share_directory('pi_gpio')
    resource_dir = os.path.join(package_share_dir, 'resource')
    builtin_config = os.path.join(resource_dir, 'io_config.txt')

    node = BuiltinGPIOActionServer(builtin_config)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
