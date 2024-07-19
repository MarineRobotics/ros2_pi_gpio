import time
import board
import adafruit_pcf8574
from mr_interfaces.action import GPIO as GPIO_Action
from .base_gpio_action_server import BaseGPIOActionServer

class PCF8574GPIOActionServer(BaseGPIOActionServer):
    def __init__(self, config_file="pcf8574_gpio_config.txt"):
        super().__init__('pcf8574_gpio_server', 'pcf8574_gpio_server', config_file)
        self.i2c = board.I2C()  # uses board.SCL and board.SDA
        self.pcf = adafruit_pcf8574.PCF8574(self.i2c)
        self.pin_objects = {}  # Store PCF8574 pin objects
        self.setup_pins()

    def setup_pin(self, pin_id, pin_type):
        pin = self.pcf.get_pin(pin_id)
        if pin_type == "in":
            pin.switch_to_input(pull=adafruit_pcf8574.Pull.UP)
        elif pin_type == "out":
            pin.switch_to_output(value=False)
        self.pin_objects[pin_id] = pin
        self.pin_types[pin_id] = pin_type

    def perform_gpio_action(self, pin_id, action_type):
        result = GPIO_Action.Result()
        pin_id = int(pin_id)

        if pin_id not in self.pin_objects:
            self.get_logger().warn(f'Invalid pin {pin_id}')
            result.value = -1
            return result.value

        pin = self.pin_objects[pin_id]

        if action_type == "high" and self.pin_types[pin_id] == "out":
            pin.value = True
            time.sleep(0.1)
            result.value = 3
        elif action_type == "low" and self.pin_types[pin_id] == "out":
            pin.value = False
            time.sleep(0.1)
            result.value = 3
        elif action_type == "read":
            result.value = int(pin.value)
        else:
            self.get_logger().warn(f'Invalid action {action_type} for pin {pin_id}')
            result.value = -1

        return result.value

    def destroy(self):
        # No specific cleanup needed for PCF8574
        super().destroy()
