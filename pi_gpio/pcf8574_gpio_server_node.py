import rclpy
from pi_gpio.pcf8574_gpio_server import PCF8574IO
from ament_index_python.packages import get_package_share_directory
import os

def main(args=None):
    rclpy.init(args=args)

    package_share_dir = get_package_share_directory('pi_gpio')
    resource_dir = os.path.join(package_share_dir, 'resource')
    pcf8574_config = os.path.join(resource_dir, 'pcf8574_config.txt')

    node = PCF8574IO(pcf8574_config)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
