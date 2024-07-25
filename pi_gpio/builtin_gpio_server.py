import time
import RPi.GPIO as GPIO
from rclpy.node import Node
from std_msgs.msg import Bool
from mr_interfaces.action import GPIO as GPIO_Action
from .base_gpio_action_server import BaseGPIOActionServer

class BuiltinGPIOActionServer(BaseGPIOActionServer):
    def __init__(self, config_file="builtin_gpio_config.txt"):
        super().__init__('pi_gpio_server', 'pi_gpio_server', config_file)
        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BCM)
        self.pin_publishers = {}
        self.setup_pins()
        
        # Create a timer that calls the publish_pin_states method 10 times per second
        self.create_timer(0.1, self.publish_pin_states)

    def setup_pin(self, pin_id, pin_type):
        pin_id = int(pin_id)
        if pin_type == "in":
            GPIO.setup(pin_id, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
            self.pin_publishers[pin_id] = self.create_publisher(Bool, f'pin_state_{pin_id}', 1)
        elif pin_type == "out":
            GPIO.setup(pin_id, GPIO.OUT)
        self.pin_types[pin_id] = pin_type

    def publish_pin_states(self):
        for pin_id, pin_type in self.pin_types.items():
            if pin_type == "in":
                state = GPIO.input(pin_id)
                self.pin_publishers[pin_id].publish(Bool(data=bool(state)))

    def perform_gpio_action(self, pin_id, action_type):
        result = GPIO_Action.Result()
        pin_id = int(pin_id)

        if pin_id not in self.pin_types or self.pin_types[pin_id] != "out":
            self.get_logger().warn(f'Invalid output pin {pin_id}')
            result.value = -1
            return result.value

        if action_type == "high":
            GPIO.output(pin_id, GPIO.HIGH)
            result.value = 1
        elif action_type == "low":
            GPIO.output(pin_id, GPIO.LOW)
            result.value = 0
        elif action_type == "read":
            result.value = GPIO.input(pin_id)
        else:
            self.get_logger().warn(f'Invalid action {action_type} for pin {pin_id}')
            result.value = -1

        return result.value

    def destroy(self):
        GPIO.cleanup()
        super().destroy()
