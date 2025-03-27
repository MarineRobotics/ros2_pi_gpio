#!/usr/bin/env python3

import board
import adafruit_pcf8574
import digitalio
#import rclpy
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import Bool

from pi_gpio.base_gpio_server import BaseGPIOServer

class PCF8574Server(BaseGPIOServer):
    """
    GPIO server implementation for PCF8574 I2C GPIO expander.
    Handles reading and writing to PCF8574 pins.
    """
    def __init__(self, config_file="pcf8574_config.yaml"):
        """
        Initialize the PCF8574 GPIO server.
        
        Args:
            config_file (str): Path to the configuration file
        """
        super().__init__('pcf8574_server', config_file)
        
        # Initialize I2C and PCF8574
        try:
            i2c = board.I2C()
            self.pcf = adafruit_pcf8574.PCF8574(i2c)
            self.get_logger().info('PCF8574 initialized successfully')
        except Exception as e:
            self.get_logger().error(f'Failed to initialize PCF8574: {e}')
            rclpy.shutdown()
            raise
    
        # Setup pins and create timer for publishing input states
        # polling rate and publish_pin_states are defined in the base class
        self.setup_pins()
        self.create_timer(1.0/self.polling_rate, self.publish_pin_states)
 
    def setup_pin(self, pin_id, pin_type):
        """
        Configure a PCF8574 pin as input or output.
        
        Args:
            pin_id (int): Pin identifier (0-7)
            pin_type (str): Pin type ('in' or 'out')
        """
        pin_id = int(pin_id)
        try:
            pin_obj = self.pcf.get_pin(pin_id)
            
            if pin_type == "in":
                pin_obj.switch_to_input(pull=digitalio.Pull.UP)
                self.pin_publishers[pin_id] = self.create_publisher(
                    Bool, 
                    f'external_gpio_{pin_id}', 
                    10
                )
                self.input_pins.append(pin_id)
                self.get_logger().info(f'Pin {pin_id} configured as input')
            
            elif pin_type == "out":
                pin_obj.switch_to_output(value=False)
                self.output_pins.append(pin_id)
                self.get_logger().info(f'Pin {pin_id} configured as output')
            
            self.pin_types[pin_id] = pin_type
        
        except Exception as e:
            self.get_logger().error(f'Error setting up pin {pin_id}: {e}')
    
    def read_pin(self, pin_id):
        """
        Read the current state of a PCF8574 pin.
        
        Args:
            pin_id (int): Pin identifier
            
        Returns:
            int: Pin state (1 for HIGH, 0 for LOW), or -1 if error
        """
        pin_id = int(pin_id)
        try:
            pin_obj = self.pcf.get_pin(pin_id)
            # self.get_logger().debug(f'Reading pin {pin_id}, value={pin_obj.value}', throttle_duration_sec=1)
            # For input pins, we invert the value because of pull-up resistors
            if pin_id in self.input_pins:
                return 1 if not pin_obj.value else 0
            else:
                # For output pins, we can read back the current state directly
                # PCF8574 allows reading the state of pins configured as outputs
                return 1 if pin_obj.value else 0
        except Exception as e:
            self.get_logger().error(f'Error reading pin {pin_id}: {e}')
            return -1
    
    def write_pin(self, pin_id, value):
        """
        Write a value to a PCF8574 output pin.
        
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
            
            pin_obj = self.pcf.get_pin(pin_id)
            pin_obj.value = bool(value)
            
            # We successfully sent the command to the hardware
            self.get_logger().debug(f'Set pin {pin_id} to {value}')
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
        except Exception as e:
            self.get_logger().error(f'Error during shutdown: {e}')
        
        # Call parent class destroy
        super().destroy()

def main(args=None):
    """
    Main function to run the PCF8574 GPIO server.
    """
    rclpy.init(args=args)
    
    try:
        # Create the PCF8574 GPIO server node
        pcf8574_server = PCF8574Server()
        
        # Use a MultiThreadedExecutor for better performance
        executor = MultiThreadedExecutor()
        executor.add_node(pcf8574_server)
        
        try:
            pcf8574_server.get_logger().info('PCF8574 GPIO server running')
            executor.spin()
        except KeyboardInterrupt:
            pcf8574_server.get_logger().info('Keyboard interrupt received')
        finally:
            pcf8574_server.destroy()
            executor.shutdown()
            pcf8574_server.get_logger().info('PCF8574 GPIO server stopped')
    
    except Exception as e:
        print(f'Error: {e}')
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()
