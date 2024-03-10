import unittest
from unittest.mock import patch

from nav_msgs.msg import OccupancyGrid

from src.common.types import CoordinatesTuple, Grid
from src.services.astar_service import AStarService


class TestAStarService(unittest.TestCase):
    def setUp(self):
        self.goal_position = (4, 4)
        self.service = AStarService(goal_position=self.goal_position)

    def create_mock_occupancy_grid(
        self, width: int, height: int, data: list
    ) -> OccupancyGrid:
        occupancy_grid = OccupancyGrid()
        occupancy_grid.info.width = width
        occupancy_grid.info.height = height
        occupancy_grid.data = data
        return occupancy_grid

    def test_convert_occupancy_grid_to_grid(self):
        mock_occupancy_grid = self.create_mock_occupancy_grid(
            width=5,
            height=5,
            data=[0] * 20 + [100] * 5,
        )
        expected_grid = [[0] * 5 for _ in range(4)] + [[1] * 5]

        grid = self.service._AStarService__convert_occupancy_grid_to_grid(
            mock_occupancy_grid
        )

        self.assertEqual(grid, expected_grid)

    def test_astar_search_simple_path(self):
        start = (0, 0)
        self.service._AStarService__goal_position = (2, 2)

        mock_occupancy_grid = self.create_mock_occupancy_grid(3, 3, [0] * 9)

        path = self.service.run_astar(mock_occupancy_grid, (0, 0))
        expected_path = [(0, 0), (0, 1), (0, 2), (1, 2), (2, 2)]

        self.assertEqual(path, expected_path)

    @patch.object(AStarService, "_AStarService__convert_occupancy_grid_to_grid")
    @patch.object(AStarService, "_AStarService__astar_search")
    def test_run_astar_integration(self, mock_astar_search, mock_convert_grid):
        mock_occupancy_grid = self.create_mock_occupancy_grid(5, 5, [0] * 25)
        start = (0, 0)

        mock_convert_grid.return_value = [[0] * 5 for _ in range(5)]
        mock_astar_search.return_value = [
            (0, 0),
            (4, 4),
        ]

        path = self.service.run_astar(mock_occupancy_grid, start)

        mock_convert_grid.assert_called_once_with(mock_occupancy_grid)
        mock_astar_search.assert_called_once()
        self.assertEqual(path, [(0, 0), (4, 4)])


if __name__ == "__main__":
    unittest.main()
