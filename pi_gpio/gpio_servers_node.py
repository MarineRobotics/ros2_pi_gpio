import rclpy
from rclpy.executors import MultiThreadedExecutor
from pi_gpio.builtin_gpio_server import BuiltinGPIOActionServer
from pi_gpio.pcf8574_gpio_server import PCF8574GPIOActionServer

def main(args=None):
    rclpy.init(args=args)

    builtin_server = BuiltinGPIOActionServer()
    pcf8574_server = PCF8574GPIOActionServer()

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
