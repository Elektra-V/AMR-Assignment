import unittest
from unittest.mock import MagicMock, patch

from nav_msgs.msg import OccupancyGrid, Odometry
from sensor_msgs.msg import LaserScan

from src.common.constants import Constants
from src.common.types import Coordinates
from src.services.astar_service import AStarService
from src.services.path_planning_service import PathPlanningService
from src.services.potential_field_service import PotentialFieldService
from src.utils.event_bus import EventBus


class TestPathPlanningService(unittest.TestCase):
    def setUp(self):
        self.event_bus = EventBus()
        self.event_bus.publish = MagicMock()

        self.initial_position = Coordinates(x=0, y=0)
        self.goal_position = Coordinates(x=10, y=10)

        self.service = PathPlanningService(
            event_bus=self.event_bus,
            initial_position=self.initial_position,
            goal_position=self.goal_position,
        )

    @patch.object(AStarService, "run_astar")
    @patch.object(PotentialFieldService, "run_potential_field")
    def test_path_planning_integration(self, mock_run_potential_field, mock_run_astar):
        mock_run_astar.return_value = [(0, 0), (5, 5), (10, 10)]
        mock_run_potential_field.return_value = MagicMock(
            linear_velocity_x=1.0, angular_velocity_z=0.0
        )

        mock_occupancy_grid = MagicMock(spec=OccupancyGrid)
        mock_odometry = MagicMock(spec=Odometry)
        mock_laser_scan = MagicMock(spec=LaserScan)

        self.service.run_path_planner(mock_occupancy_grid)
        self.service.run_path_planner(mock_odometry)
        self.service.run_path_planner(mock_laser_scan)

        updated_position = (5, 5)

        mock_run_astar.assert_called_once_with(
            mock_occupancy_grid, self.initial_position
        )
        mock_run_potential_field.assert_called_once_with(
            updated_position,
            self.goal_position,
            mock_odometry,
            mock_laser_scan,
        )

        self.event_bus.publish.assert_called_with(
            Constants.EVENT_TOPIC_MOVEMENT, mock_run_potential_field.return_value
        )


if __name__ == "__main__":
    unittest.main()
