import rclpy
from pi_gpio.builtin_gpio_server import BuiltinGPIOServer
from ament_index_python.packages import get_package_share_directory
import os

def main(args=None):
    rclpy.init(args=args)

    # Get configuration file path
    package_share_dir = get_package_share_directory('pi_gpio')
    resource_dir = os.path.join(package_share_dir, 'resource')
    builtin_config = os.path.join(resource_dir, 'io_config.txt')

    try:
        # Create the Raspberry Pi GPIO server node
        builtin_server = BuiltinGPIOServer(builtin_config)

        try:
            builtin_server.get_logger().info('Raspberry Pi GPIO server running')
            rclpy.spin(builtin_server)  # Use single-threaded spin instead of executor
        except KeyboardInterrupt:
            builtin_server.get_logger().info('Keyboard interrupt received')
        finally:
            builtin_server.destroy()
            builtin_server.get_logger().info('Raspberry Pi GPIO server stopped') 
    except Exception as e:
        print(f'Error: {e}')
    finally:
        rclpy.shutdown()
