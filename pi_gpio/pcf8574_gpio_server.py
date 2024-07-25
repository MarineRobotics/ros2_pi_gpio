import board
import adafruit_pcf8574
import digitalio
from mr_interfaces.action import GPIO as GPIO_ACTION
from .base_gpio_action_server import BaseGPIOActionServer

class PCF8574IO(BaseGPIOActionServer):
    def __init__(self, config_file="pcf8574_config.txt"):
        super().__init__('pcf8574_io', 'pcf8574_gpio_server', config_file)
        
        # Initialize I2C and PCF8574
        i2c = board.I2C()
        self.pcf = adafruit_pcf8574.PCF8574(i2c)
        
        self.setup_pins()
        
        # Create a timer that calls the publish_pin_states method 10 times per second
        self.create_timer(0.1, self.publish_pin_states)

    def setup_pin(self, pin_id, pin_type):
        pin_id = int(pin_id)
        pin_obj = self.pcf.get_pin(pin_id)
        
        if pin_type == "in":
            pin_obj.switch_to_input(pull=digitalio.Pull.UP)
            self.pin_publishers[pin_id] = self.create_publisher(GPIO_ACTION, f'external_gpio_{pin_id}', 1)
        elif pin_type == "out":
            pin_obj.switch_to_output(value=False)
        
        self.pin_types[pin_id] = pin_type

    def publish_pin_states(self):
        for pin_id, pin_type in self.pin_types.items():
            if pin_type == "in":
                pin_obj = self.pcf.get_pin(pin_id)
                state = not pin_obj.value  # Invert because input is pulled up
                msg = GPIO_ACTION()
                msg.gpio = f"{pin_id},{int(state)}"
                self.pin_publishers[pin_id].publish(msg)

    def perform_gpio_action(self, pin_id, action_type):
        pin_id = int(pin_id)
        result = GPIO_ACTION.Result()

        if pin_id not in self.pin_types or self.pin_types[pin_id] != "out":
            self.get_logger().warn(f'Invalid output pin {pin_id}')
            result.value = -1
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
            result.value = -1

        return result.value  # Return just the value, not the whole result object

    def destroy(self):
        # Perform any necessary cleanup
        super().destroy()
