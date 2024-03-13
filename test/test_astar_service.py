import unittest
from unittest.mock import MagicMock

from nav_msgs.msg import OccupancyGrid

from src.common.types import CoordinatesTuple
from src.services.astar_service import AStarService


class TestAStarService(unittest.TestCase):
    def setUp(self):
        self.astar_service = AStarService()

    def test_run_astar(self):
        mock_map = MagicMock(spec=OccupancyGrid)
        mock_map.info.width = 5
        mock_map.info.height = 5
        mock_map.data = [0] * 25
        mock_map.info.resolution = 1.0
        mock_map.info.origin.position.x = 0
        mock_map.info.origin.position.y = 0

        current_position = (2.0, 2.0)
        goal_position = (4.0, 4.0)

        expected_path_world = [(3.0, 3.0), (4.0, 4.0)]

        path_world = self.astar_service.run_astar(
            current_position, goal_position, mock_map
        )

        self.assertEqual(path_world, expected_path_world)


if __name__ == "__main__":
    unittest.main()
