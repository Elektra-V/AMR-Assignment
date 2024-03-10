import unittest
from unittest.mock import MagicMock

import rclpy
from nav_msgs.msg import OccupancyGrid, Odometry
from sensor_msgs.msg import LaserScan

from src.common.constants import Constants
from src.services.path_planning_service import PathPlanningService
from src.subscribers import SLAMSubscriber
from src.utils.event_bus import EventBus


class TestSLAMSubscriber(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.mock_path_planning_service = MagicMock(spec=PathPlanningService)
        self.node = SLAMSubscriber(
            path_planning_service=self.mock_path_planning_service
        )

    def test_map_subscription_callback(self):
        mock_msg = MagicMock(spec=OccupancyGrid)
        self.node._SLAMSubscriber__map_subscription_callback(mock_msg)
        self.mock_path_planning_service.run_path_planner.assert_called_once_with(
            mock_msg
        )

    def test_odometry_subscription_callback(self):
        mock_msg = MagicMock(spec=Odometry)
        self.node._SLAMSubscriber__odometry_subscription_callback(mock_msg)
        self.mock_path_planning_service.run_path_planner.assert_called_once_with(
            mock_msg
        )

    def test_lidar_subscription_callback(self):
        mock_msg = MagicMock(spec=LaserScan)
        self.node._SLAMSubscriber__lidar_subscription_callback(mock_msg)
        self.mock_path_planning_service.run_path_planner.assert_called_once_with(
            mock_msg
        )


if __name__ == "__main__":
    unittest.main()
