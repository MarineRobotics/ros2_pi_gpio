import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.context import Context
from pi_gpio.builtin_gpio_server import BuiltinGPIOActionServer
from pi_gpio.pcf8574_gpio_server import PCF8574IO
from ament_index_python.packages import get_package_share_directory
import os

def main(args=None):
    # Create separate contexts for each node
    builtin_context = Context()
    pcf8574_context = Context()

    rclpy.init(args=args, context=builtin_context)
    rclpy.init(args=args, context=pcf8574_context)

    # Get the path to the config files
    package_share_dir = get_package_share_directory('pi_gpio')
    resource_dir = os.path.join(package_share_dir, 'resource')
    builtin_config = os.path.join(resource_dir, 'io_config.txt')
    pcf8574_config = os.path.join(resource_dir, 'pcf8574_config.txt')

    # Create nodes with their respective contexts
    builtin_server = BuiltinGPIOActionServer(builtin_config, context=builtin_context)
    pcf8574_server = PCF8574IO(pcf8574_config, context=pcf8574_context)

    executor = MultiThreadedExecutor()
    executor.add_node(builtin_server)
    executor.add_node(pcf8574_server)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        builtin_server.destroy_node()
        pcf8574_server.destroy_node()
        rclpy.shutdown(context=builtin_context)
        rclpy.shutdown(context=pcf8574_context)

if __name__ == '__main__':
    main()
