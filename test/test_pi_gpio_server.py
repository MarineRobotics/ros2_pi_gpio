import pytest
import time
"""
Use the pip package 'fake-rpi' to mock the RPi.GPIO library.
Must be installed with pip 'pip install fake-rpi'
"""
import sys
import fake_rpi
from unittest.mock import MagicMock, patch
sys.modules['RPi'] = fake_rpi.RPi     # Fake RPi
sys.modules['RPi.GPIO'] = fake_rpi.RPi.GPIO # Fake GPIO
sys.modules['smbus'] = fake_rpi.smbus # Fake smbus (I2C)
# sys.modules['RPi'] = MagicMock()
# sys.modules['RPi.GPIO'] = MagicMock()
# sys.modules['GPIO'] = MagicMock()
# sys.modules['GPIO'].LOW = 0
# sys.modules['GPIO'].HIGH = 1
# sys.modules['GPIO'].OUT = 'out'
# sys.modules['GPIO'].IN = 'in'

import RPi.GPIO as GPIO
from pi_gpio.pi_gpio_server import GPIOActionServer, RaspberryPIGPIO
from mr_interfaces.action import GPIO as GPIO_Action
from rclpy.action import ActionClient
from rclpy.node import Node
from _pytest.monkeypatch import MonkeyPatch
import rclpy


!!!!!!!!!!!!!!!
# TODO: Only keep relevant tests (don't test gpio separately for example)
# Make sure action server is tested correctly
!!!!!!!!!!!!!!!!



# Initialize rclpy
@pytest.fixture(scope='module', autouse=True)
def init_rclpy():
    rclpy.init()
    yield
    rclpy.shutdown()

@pytest.fixture
def gpio_action_server():
    server = GPIOActionServer()
    return server

# # Mock RPi.GPIO
# @pytest.fixture(autouse=True)
# def mock_rpi_gpio():
#     with patch('RPi.GPIO') as gpio:
#         yield gpio

@pytest.fixture
def gpio_action_client():
    return ActionClient(Node(), GPIO_Action, 'pi_gpio_server')


def test_gpio_action_server_goal_callback(gpio_action_server):
    goal_request = gpio_action_server.goal_callback(None)
    assert goal_request == gpio_action_server.GoalResponse.ACCEPT


def test_gpio_action_server_cancel_callback(gpio_action_server):
    cancel_response = gpio_action_server.cancel_callback(None)
    assert cancel_response == gpio_action_server.CancelResponse.ACCEPT


def test_gpio_action_server_execute_callback(gpio_action_server, gpio_action_client):
    # Send a goal to the action server
    goal_msg = GPIO_Action.Goal()
    goal_msg.gpio = '1,high'
    future = gpio_action_client.send_goal_async(goal_msg)

    # Wait for the goal to complete
    rclpy.spin_until_future_complete(gpio_action_server, future)
    result = future.result()

    # Check the result
    assert result.value == 3


def test_raspberry_pi_gpio_init():
    pin_id = 1
    gpio_type = 'in'
    gpio = RaspberryPIGPIO(pin_id, gpio_type)
    assert gpio.pin_id == pin_id
    assert gpio.type == gpio_type


def test_raspberry_pi_gpio_set_pin():
    pin_id = 1
    gpio_type = 'out'
    gpio = RaspberryPIGPIO(pin_id, gpio_type)

    # Set pin to high
    gpio.set_pin(1)
    time.sleep(0.1)
    assert GPIO.input(pin_id) == GPIO.HIGH

    # Set pin to low
    gpio.set_pin(0)
    time.sleep(0.1)
    assert GPIO.input(pin_id) == GPIO.LOW


def test_raspberry_pi_gpio_read_pins_from_file():
    pin_list = RaspberryPIGPIO.read_pins_from_file()
    assert isinstance(pin_list, list)


if __name__ == '__main__':
    pytest.main(['-v', __file__])
