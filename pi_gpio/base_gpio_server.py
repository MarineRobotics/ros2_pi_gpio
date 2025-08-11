#!/usr/bin/env python3

import threading
import yaml
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.service import Service
from std_msgs.msg import Bool
from std_srvs.srv import Trigger
from mr_interfaces.srv import SetGPIO, ReadGPIO  # These service interfaces need to be defined

class BaseGPIOServer(Node):
    """
    Base class for GPIO servers that handles GPIO pin operations.
    This class defines the ROS2 interface but doesn't implement hardware-specific operations.
    """
    def __init__(self, node_name, config_file):
        """
        Initialize the GPIO server with the given name and configuration file.
        
        Args:
            node_name (str): Name of the ROS2 node
            config_file (str): Path to the configuration file
        """
        super().__init__(node_name)
        
        # Create callback group for concurrent service handling
        self.callback_group = ReentrantCallbackGroup()
        
        # Initialize pin tracking dictionaries
        self.pin_types = {}  # Stores pin types (in/out)
        self.input_pins = []  # List of input pin IDs
        self.output_pins = []  # List of output pin IDs
        self.pin_publishers = {}  # Dictionary of publishers for input pins
        self.pin_states = {}  # Track last known pin states
        
        # Load configuration
        self.config_file = config_file
        self.get_logger().info(f'Loading configuration from {config_file}')
        
        # Create services
        self._set_gpio_service = self.create_service(
            SetGPIO,
            f'{node_name}/set_pin',
            self.set_gpio_callback,
            callback_group=self.callback_group
        )
        
        self._read_gpio_service = self.create_service(
            ReadGPIO,
            f'{node_name}/read_pin',
            self.read_gpio_callback,
            callback_group=self.callback_group
        )
        
        self._health_check_service = self.create_service(
            Trigger,
            f'{node_name}/health_check',
            self.health_check_callback,
            callback_group=self.callback_group
        )
        
        # Declare configurable parameters
        self.declare_parameter('polling_rate', 10.0)  # Default 10Hz polling rate
        self.polling_rate = self.get_parameter('polling_rate').value
        
        # Setup pins and create timer for publishing input states
        # self.setup_pins()
        # self.create_timer(1.0/self.polling_rate, self.publish_pin_states)
        
        self.get_logger().info(f'{node_name} initialized successfully')
    
    def read_config(self):
        """
        Read pin configuration from config file.
        The format should be a YAML file with pin definitions.
        
        Returns:
            list: List of (pin_id, pin_type) tuples
        """
        try:
            with open(self.config_file, 'r') as f:
                if self.config_file.endswith('.yaml') or self.config_file.endswith('.yml'):
                    config = yaml.safe_load(f)
                    return [(str(item['pin']), item['type']) for item in config['pins']]
                else:  # Fallback to simple CSV format for backward compatibility
                    return [line.strip().split(',') for line in f if line.strip()]
        except Exception as e:
            self.get_logger().error(f'Error reading config file: {e}')
            return []
    
    def setup_pins(self):
        """
        Set up all pins defined in the configuration.
        """
        pin_list = self.read_config()
        for pin_id, pin_type in pin_list:
            self.setup_pin(int(pin_id), pin_type)
    
    def setup_pin(self, pin_id, pin_type):
        """
        Set up a single pin with the given type.
        Must be implemented by derived classes.
        
        Args:
            pin_id (int): Pin identifier
            pin_type (str): Pin type ('in' or 'out')
        """
        raise NotImplementedError("Subclasses must implement setup_pin")
    
    def read_pin(self, pin_id):
        """
        Read the current state of a pin.
        Must be implemented by derived classes.
        
        Args:
            pin_id (int): Pin identifier
            
        Returns:
            int: Pin state (0 or 1), or -1 if error
        """
        raise NotImplementedError("Subclasses must implement read_pin")
    
    def write_pin(self, pin_id, value):
        """
        Write a value to an output pin.
        Must be implemented by derived classes.
        
        Args:
            pin_id (int): Pin identifier
            value (bool): Value to write (True/False)
            
        Returns:
            bool: True if command was sent successfully, False otherwise
        """
        raise NotImplementedError("Subclasses must implement write_pin")
    
    def publish_pin_states(self):
        """
        Publish the current state of all input pins.
        This is called periodically by the timer.
        """
        # log input pins every second
        # self.get_logger().debug(f'Input pins: {self.input_pins}', throttle_duration_sec=1)
        # log pin states every second
        # self.get_logger().debug(f'Pin states: {self.pin_states}', throttle_duration_sec=1)
        for pin_id in self.input_pins:
            state = self.read_pin(pin_id)
            # Print pin id and state throttled once every 5 seconds
            # self.get_logger().debug(f'Pin {pin_id} state: {state}', throttle_duration_sec=5)
                # Only publish if the state has changed or we haven't published before
                # TODO: we've disabled this. Seems to be a better idea to publish all at a slow interval
                # if pin_id not in self.pin_states or self.pin_states[pin_id] != state:
            self.pin_states[pin_id] = state
            # Convert to boolean for publishing
            bool_state = bool(state) if state >= 0 else False
            # self.get_logger().debug(f'Publishing pin {pin_id} state: {bool_state}')
            self.pin_publishers[pin_id].publish(Bool(data=bool_state))
    
    def set_gpio_callback(self, request, response):
        """
        ROS2 service callback to set a GPIO pin state.
        
        Args:
            request: Service request containing pin_id and value
            response: Service response
            
        Returns:
            response: Service response with success flag and message
        """
        pin_id = int(request.pin_id)
        value = bool(request.value)
        
        self.get_logger().debug(f'Setting pin {pin_id} to {value}')
        
        if pin_id not in self.output_pins:
            response.success = False
            response.message = f'Pin {pin_id} is not configured as an output pin'
            return response
        
        success = self.write_pin(pin_id, value)
        
        if success:
            response.success = True
            response.message = f'Pin {pin_id} was set to {value}'
        else:
            response.success = False
            response.message = f'Failed to set pin {pin_id}'
        
        return response
    
    def read_gpio_callback(self, request, response):
        """
        ROS2 service callback to read a GPIO pin state.
        
        Args:
            request: Service request containing pin_id
            response: Service response
            
        Returns:
            response: Service response with value, success flag, and message
        """
        pin_id = int(request.pin_id)
        self.get_logger().debug(f'Reading pin {pin_id}')
        
        if pin_id not in self.input_pins and pin_id not in self.output_pins:
            response.success = False
            response.message = f'Pin {pin_id} is not configured'
            response.value = -1
            return response
        
        value = self.read_pin(pin_id)
        
        if value >= 0:
            response.success = True
            response.value = value
            response.message = f'Pin {pin_id} value is {value}'
        else:
            response.success = False
            response.value = -1
            response.message = f'Failed to read pin {pin_id}'
        
        return response
    
    def health_check_callback(self, request, response):
        """
        ROS2 service callback to check if the GPIO server is healthy.
        
        Args:
            request: Empty service request
            response: Service response
            
        Returns:
            response: Service response with success flag and message
        """
        # Simple health check - could be expanded with real diagnostics
        response.success = True
        response.message = f'{self.get_name()} is running'
        return response
    
    def destroy(self):
        """
        Clean up resources before node shutdown.
        """
        self.get_logger().info('Shutting down GPIO server')
        # Derived classes should call super().destroy() after their cleanup
