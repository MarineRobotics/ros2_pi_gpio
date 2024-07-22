import rclpy
from rclpy.executors import MultiThreadedExecutor
from pi_gpio.builtin_gpio_server import BuiltinGPIOActionServer
from pi_gpio.pcf8574_gpio_server import PCF8574IO
from ament_index_python.packages import get_package_share_directory
import os

def main(args=None):
    rclpy.init(args=args)
    
    # Get the path to the config files
    config_dir = get_package_share_directory('pi_gpio')
    builtin_config = os.path.join(config_dir, 'io_config.txt')
    pcf8574_config = os.path.join(config_dir, 'pcf8574_config.txt')

    builtin_server = BuiltinGPIOActionServer(builtin_config)
    pcf8574_server = PCF8574IO(pcf8574_config)

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
        rclpy.shutdown()

if __name__ == '__main__':
    main()
