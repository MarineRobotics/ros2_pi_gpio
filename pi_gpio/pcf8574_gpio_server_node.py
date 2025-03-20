import rclpy
from pi_gpio.pcf8574_gpio_server import PCF8574IO
from ament_index_python.packages import get_package_share_directory
import os

def main(args=None):
    rclpy.init(args=args)
    
    # Get configuration file path
    package_share_dir = get_package_share_directory('pi_gpio')
    resource_dir = os.path.join(package_share_dir, 'resource')
    pcf8574_config = os.path.join(resource_dir, 'pcf8574_config.txt')
    
    try:
        # Create the PCF8574 GPIO server node
        pcf8574_server = PCF8574Server(pcf8574_config)
        
        try:
            pcf8574_server.get_logger().info('PCF8574 GPIO server running')
            rclpy.spin(pcf8574_server)  # Use single-threaded spin instead of executor
        except KeyboardInterrupt:
            pcf8574_server.get_logger().info('Keyboard interrupt received')
        finally:
            pcf8574_server.destroy()
            pcf8574_server.get_logger().info('PCF8574 GPIO server stopped')
    
    except Exception as e:
        print(f'Error: {e}')
    finally:
        rclpy.shutdown()
