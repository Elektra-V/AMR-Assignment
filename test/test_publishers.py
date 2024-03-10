import unittest
from unittest.mock import MagicMock

import rclpy
from geometry_msgs.msg import Twist

from src.publishers import SLAMPublisher
from src.utils.event_bus import EventBus


class TestSLAMPublisher(unittest.TestCase):
    def setUp(self):
        self.mock_event_bus = MagicMock(spec=EventBus)
        self.publisher = SLAMPublisher(event_bus=self.mock_event_bus)

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def test_publish_velocity(self):
        self.publisher._SLAMPublisher__publisher = MagicMock()

        movement_input = (1.0, 0.5)  # Example linear_x and angular_z values
        self.publisher._SLAMPublisher__publish_velocity(movement_input)

        expected_msg = Twist()
        expected_msg.linear.x = 1.0
        expected_msg.angular.z = 0.5

        self.publisher._SLAMPublisher__publisher.publish.assert_called_once_with(
            expected_msg
        )

        published_msg = self.publisher._SLAMPublisher__publisher.publish.call_args[0][0]
        self.assertEqual(published_msg.linear.x, expected_msg.linear.x)
        self.assertEqual(published_msg.angular.z, expected_msg.angular.z)


if __name__ == "__main__":
    unittest.main()
