import time
import RPi.GPIO as GPIO
from mr_interfaces.action import GPIO as GPIO_Action
from .base_gpio_action_server import BaseGPIOActionServer

class BuiltinGPIOActionServer(BaseGPIOActionServer):
    def __init__(self, config_file="builtin_gpio_config.txt"):
        super().__init__('pi_gpio_server', 'pi_gpio_server', config_file)
        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BCM)
        self.pin_types = {}  # Store pin types (in/out) instead of RaspberryPIGPIO objects
        self.setup_pins()

    def setup_pin(self, pin_id, pin_type):
        if pin_type == "in":
            GPIO.setup(pin_id, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
        elif pin_type == "out":
            GPIO.setup(pin_id, GPIO.OUT)
        self.pin_types[pin_id] = pin_type

    def perform_gpio_action(self, pin_id, action_type):
        result = GPIO_Action.Result()
        pin_id = int(pin_id)

        if action_type == "high" and self.pin_types[pin_id] == "out":
            GPIO.output(pin_id, GPIO.HIGH)
            time.sleep(0.1)
            result.value = 3
        elif action_type == "low" and self.pin_types[pin_id] == "out":
            GPIO.output(pin_id, GPIO.LOW)
            time.sleep(0.1)
            result.value = 3
        elif action_type == "read":
            result.value = GPIO.input(pin_id)
        else:
            self.get_logger().warn(f'Invalid action {action_type} for pin {pin_id}')
            result.value = -1

        return result.value

    def destroy(self):
        GPIO.cleanup()
        super().destroy()
