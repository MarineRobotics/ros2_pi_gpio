import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import Bool
from mr_interfaces.action import GPIO
import board
import adafruit_pcf8574
import digitalio
import time

class PCF8574IO(Node):
    def __init__(self, config_file="pcf8574_config.txt"):
        super().__init__('pcf8574_io')
        
        # Initialize I2C and PCF8574
        i2c = board.I2C()
        self.pcf = adafruit_pcf8574.PCF8574(i2c)
        
        # Read configuration
        self.input_pins, self.output_pins = self.read_config(config_file)
        
        # Set up input pins and create publishers
        self.pin_publishers = {}
        
        for pin, topic in self.input_pins.items():
            pin_obj = self.pcf.get_pin(pin)
            pin_obj.switch_to_input(pull=digitalio.Pull.UP)
            self.pin_publishers[pin] = self.create_publisher(Bool, f'{topic}', 1)
        
        # Set up output pins
        for pin in self.output_pins:
            pin_obj = self.pcf.get_pin(pin)
            pin_obj.switch_to_output(value=False)
        
        # Create a timer that calls the publish_pin_states method 10 times per second
        self.create_timer(0.1, self.publish_pin_states)
        
        # Set up action server for output control
        self._action_server = ActionServer(
            self,
            GPIO,
            'pcf8574_gpio_server',
            self.execute_callback,
            callback_group=ReentrantCallbackGroup())

    def read_config(self, config_file):
        input_pins = {}
        output_pins = []
        try:
            with open(config_file, 'r') as f:
                for line in f:
                    pin, pin_type, topic = line.strip().split(',')
                    pin = int(pin)
                    if pin_type.lower() in ['input', 'in']:
                        # Add topic and pin to input pin dictionary
                        input_pins[pin] = topic
                    elif pin_type.lower() in ['output', 'out']:
                        output_pins.append(pin)
                    else:
                        self.get_logger().warn(f'Invalid pin type {pin_type} for pin {pin}')
        except FileNotFoundError:
            self.get_logger().error(f'Config file {config_file} not found')
            raise
        return input_pins, output_pins

    def publish_pin_states(self):
        for pin in self.input_pins:
            pin_obj = self.pcf.get_pin(pin)
            state = not pin_obj.value  # Invert because input is pulled up
            self.pin_publishers[pin].publish(Bool(data=state))

    def execute_callback(self, goal_handle):
        request = goal_handle.request
        pin_id, action_type = request.gpio.split(',')
        pin_id = int(pin_id)
        
        result = GPIO.Result()
        
        if pin_id not in self.output_pins:
            self.get_logger().warn(f'Invalid output pin {pin_id}')
            goal_handle.abort()
            return result
        
        pin = self.pcf.get_pin(pin_id)
        
        if action_type == "high":
            pin.value = True
            result.value = 1
        elif action_type == "low":
            pin.value = False
            result.value = 0
        elif action_type == "read":
            result.value = int(not pin.value)  # Invert the value for consistency with input pins
        else:
            self.get_logger().warn(f'Invalid action {action_type} for pin {pin_id}')
            goal_handle.abort()
            return result
        
        goal_handle.succeed()
        return result
