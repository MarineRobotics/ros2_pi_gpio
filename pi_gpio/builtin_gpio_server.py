#!/usr/bin/env python3

import RPi.GPIO as GPIO
import rclpy
from std_msgs.msg import Bool

from .base_gpio_server import BaseGPIOServer

class BuiltinGPIOServer(BaseGPIOServer):
    """
    GPIO server implementation for Raspberry Pi's built-in GPIO pins.
    Uses RPi.GPIO library to interact with the hardware.
    """
    def __init__(self, config_file="config/builtin_gpio_config.yaml"):
        """
        Initialize the Raspberry Pi GPIO server.
        
        Args:
            config_file (str): Path to the configuration file
        """
        super().__init__('gpio_server', config_file)
        
        # Initialize RPi.GPIO
        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BCM)
        self.get_logger().info('RPi.GPIO initialized in BCM mode')
    
    def setup_pin(self, pin_id, pin_type):
        """
        Configure a Raspberry Pi GPIO pin as input or output.
        
        Args:
            pin_id (int): Pin identifier
            pin_type (str): Pin type ('in' or 'out')
        """
        pin_id = int(pin_id)
        try:
            if pin_type == "in":
                GPIO.setup(pin_id, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
                self.pin_publishers[pin_id] = self.create_publisher(
                    Bool, 
                    f'gpio_{pin_id}', 
                    10
                )
                self.input_pins.append(pin_id)
                self.get_logger().info(f'Pin {pin_id} configured as input with pull-down')
            
            elif pin_type == "out":
                # Initialize outputs to LOW for safety and predictable startup state
                # This ensures all controlled devices begin in a de-energized state
                GPIO.setup(pin_id, GPIO.OUT, initial=GPIO.LOW)
                self.output_pins.append(pin_id)
                self.get_logger().info(f'Pin {pin_id} configured as output (initial LOW)')
            
            self.pin_types[pin_id] = pin_type
        
        except Exception as e:
            self.get_logger().error(f'Error setting up pin {pin_id}: {e}')
    
    def read_pin(self, pin_id):
        """
        Read the current state of a Raspberry Pi GPIO pin.
        
        Args:
            pin_id (int): Pin identifier
            
        Returns:
            int: Pin state (1 for HIGH, 0 for LOW), or -1 if error
        """
        pin_id = int(pin_id)
        try:
            if pin_id not in self.input_pins and pin_id not in self.output_pins:
                self.get_logger().warn(f'Pin {pin_id} is not configured')
                return -1
            
            # RPi.GPIO allows reading both input and output pins
            value = GPIO.input(pin_id)
            return 1 if value else 0
        
        except Exception as e:
            self.get_logger().error(f'Error reading pin {pin_id}: {e}')
            return -1
    
    def write_pin(self, pin_id, value):
        """
        Write a value to a Raspberry Pi GPIO output pin.
        
        Args:
            pin_id (int): Pin identifier
            value (bool): Value to write (True for HIGH, False for LOW)
            
        Returns:
            bool: True if command was sent successfully, False otherwise
        """
        pin_id = int(pin_id)
        try:
            if pin_id not in self.output_pins:
                self.get_logger().warn(f'Pin {pin_id} is not configured as an output pin')
                return False
            
            # Convert value to GPIO.HIGH or GPIO.LOW
            gpio_value = GPIO.HIGH if value else GPIO.LOW
            GPIO.output(pin_id, gpio_value)
            
            # We successfully sent the command to the hardware
            self.get_logger().debug(f'Set pin {pin_id} to {"HIGH" if value else "LOW"}')
            return True
        
        except Exception as e:
            self.get_logger().error(f'Error writing to pin {pin_id}: {e}')
            return False
    
    def destroy(self):
        """
        Clean up resources before node shutdown.
        """
        # Set all output pins to LOW
        try:
            for pin_id in self.output_pins:
                self.write_pin(pin_id, False)
                self.get_logger().info(f'Set pin {pin_id} to LOW during shutdown')
            
            # Clean up GPIO
            GPIO.cleanup()
            self.get_logger().info('GPIO resources cleaned up')
        except Exception as e:
            self.get_logger().error(f'Error during shutdown: {e}')
        
        # Call parent class destroy
        super().destroy()
