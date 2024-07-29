import board
import adafruit_pcf8574
import digitalio
from mr_interfaces.action import GPIO as GPIO
from std_msgs.msg import Bool
from pi_gpio.base_gpio_action_server import BaseGPIOActionServer

class PCF8574IO(BaseGPIOActionServer):
    def __init__(self, config_file="pcf8574_config.txt"):
        super().__init__('pcf8574_io', 'pcf8574_gpio_server', config_file)
        
        # Initialize I2C and PCF8574
        i2c = board.I2C()
        self.pcf = adafruit_pcf8574.PCF8574(i2c)
        
        self.pin_publishers = {}
        self.input_pins = []
        self.output_pins = []
        
        self.setup_pins()
        
        # Create a timer that calls the publish_pin_states method 10 times per second
        self.create_timer(0.1, self.publish_pin_states)

    def setup_pin(self, pin_id, pin_type):
        pin_id = int(pin_id)
        pin_obj = self.pcf.get_pin(pin_id)
        
        if pin_type == "in":
            pin_obj.switch_to_input(pull=digitalio.Pull.UP)
            self.pin_publishers[pin_id] = self.create_publisher(Bool, f'external_gpio_{pin_id}', 1)
            self.input_pins.append(pin_id)
        elif pin_type == "out":
            pin_obj.switch_to_output(value=False)
            self.output_pins.append(pin_id)
        
        self.pin_types[pin_id] = pin_type

    def publish_pin_states(self):
        for pin_id in self.input_pins:
            pin_obj = self.pcf.get_pin(pin_id)
            state = not pin_obj.value  # Invert because input is pulled up
            self.pin_publishers[pin_id].publish(Bool(data=state))

    def perform_gpio_action(self, pin_id, action_type):
        pin_id = int(pin_id)

        if pin_id not in self.output_pins:
            self.get_logger().warn(f'Invalid output pin {pin_id}')
            return -1

        pin = self.pcf.get_pin(pin_id)

        if action_type == "high":
            pin.value = True
            return 1
        elif action_type == "low":
            pin.value = False
            return 0
        elif action_type == "read":
            return int(not pin.value)  # Invert the value for consistency with input pins
        else:
            self.get_logger().warn(f'Invalid action {action_type} for pin {pin_id}')
            return -1 

    def destroy(self):
        # Perform any necessary cleanup
        super().destroy()
