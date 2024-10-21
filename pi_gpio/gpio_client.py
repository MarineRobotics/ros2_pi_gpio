import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from mr_interfaces.action import GPIO
import asyncio
from concurrent.futures import ThreadPoolExecutor

class GPIOActionClient:
    def __init__(self, node: Node):
        self._node = node
        self._action_client = ActionClient(self._node, GPIO, 'pi_gpio_server')
        self._executor = ThreadPoolExecutor(max_workers=1)
        

    def send_gpio_request(self, pin_id, action_type):
        goal_msg = GPIO.Goal()
        goal_msg.gpio = f"{pin_id},{action_type}"
        
        # Wait for action server
        if not self._action_client.wait_for_server(timeout_sec=1.0):
            self._node.get_logger().error('Action server not available')
            return False
                
        self._node.get_logger().info(f'Sending goal request: {goal_msg.gpio}')
        
        # Send goal with callback for the response
        send_goal_future = self._action_client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(self._goal_response_callback)
        
        return True

    def _goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self._node.get_logger().info('Goal rejected')
            return

        # Get the result using another callback
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._result_callback)

    def _result_callback(self, future):
        result = future.result().result
        self._node.get_logger().info(f'Got result: {result.value}')







    def set_gpio(self, pin_id, state):
        return self.send_gpio_request(pin_id, 'high' if state else 'low')
        
    def read_gpio(self, pin_id):
        return self.send_gpio_request(pin_id, 'read')
