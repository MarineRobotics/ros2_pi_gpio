import threading
import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from mr_interfaces.action import GPIO as GPIO_Action

class BaseGPIOActionServer(Node):
    def __init__(self, node_name, action_name, config_file):
        super().__init__(node_name)
        self._goal_handle = None
        self._goal_lock = threading.Lock()
        self.config_file = config_file
        self.pin_types = {}

        self._action_server = ActionServer(
            self,
            GPIO_Action,
            action_name,
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            handle_accepted_callback=self.handle_accepted_callback,
            cancel_callback=self.cancel_callback,
            callback_group=ReentrantCallbackGroup())

    @staticmethod
    def read_pins_from_file(config_file):
        with open(config_file, "r") as f:
            return [line.strip().split(',') for line in f if line.strip()]

    def setup_pins(self):
        pin_list = self.read_pins_from_file(self.config_file)
        for pin_id, pin_type in pin_list:
            self.setup_pin(int(pin_id), pin_type)

    def setup_pin(self, pin_id, pin_type):
        raise NotImplementedError("Subclasses must implement setup_pin")

    def perform_gpio_action(self, pin_id, action_type):
        raise NotImplementedError("Subclasses must implement perform_gpio_action")

    def goal_callback(self, goal_request):
        self.get_logger().info('Received goal request')
        return GoalResponse.ACCEPT

    def handle_accepted_callback(self, goal_handle):
        with self._goal_lock:
            if self._goal_handle is not None and self._goal_handle.is_active:
                self.get_logger().info('Aborting previous goal')
                self._goal_handle.abort()
            self._goal_handle = goal_handle
        goal_handle.execute()

    def cancel_callback(self, goal):
        self.get_logger().info('Received cancel request')
        return CancelResponse.ACCEPT

    def execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal...')

        goal_msg = goal_handle.request.gpio
        feedback_msg = GPIO_Action.Feedback()
        result = GPIO_Action.Result()

        if not goal_handle.is_active:
            self.get_logger().info('Goal aborted')
            return result

        if goal_handle.is_cancel_requested:
            goal_handle.canceled()
            self.get_logger().info('Goal canceled')
            return result

        # Parse the goal message
        pin_id, action_type = goal_msg.split(',')

        # This is where perform_gpio_action is called
        result.value = self.perform_gpio_action(pin_id, action_type)

        # Publish feedback (optional)
        feedback_msg.feedback = 1  # You can customize this
        goal_handle.publish_feedback(feedback_msg)

        goal_handle.succeed()
        return result
