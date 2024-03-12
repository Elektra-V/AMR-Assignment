import unittest
from os import getcwd, path
from unittest.mock import patch

from nav_msgs.msg import OccupancyGrid

from src.services.astar_service import AStarService


class TestAStarService(unittest.TestCase):
    def setUp(self):
        self.goal_position = (4, 4)
        self.service = AStarService()

        occupancy_grid = OccupancyGrid()

        with open(path.join(getcwd(), "maps", "occupancy-grid.txt"), "r") as file:
            file_lines = file.readlines()
            for i, line in enumerate(file_lines):
                match i:
                    case 0:
                        occupancy_grid.info.width = int(line)
                    case 1:
                        occupancy_grid.info.height = int(line)
                    case 2:
                        occupancy_grid.info.resolution = float(line)
                    case 3:
                        occupancy_grid.info.origin.position.x = float(line)
                    case 4:
                        occupancy_grid.info.origin.position.y = float(line)
                    case 5:
                        occupancy_grid.data = list(
                            map(lambda x: int(x), line.split(", "))
                        )

        self.__occupancy_grid = occupancy_grid
        print(self.__occupancy_grid)

    def test_convert_occupancy_grid_to_grid(self):
        mock_occupancy_grid = self.__occupancy_grid
        expected_grid = [[0] * 5 for _ in range(4)] + [[1] * 5]

        grid = self.service._AStarService__convert_occupancy_grid_to_grid(
            mock_occupancy_grid
        )

        self.assertEqual(grid, expected_grid)

    def test_astar_search_simple_path(self):
        start = (0, 0)
        self.service._AStarService__goal_position = (2, 2)

        mock_occupancy_grid = self.__occupancy_grid

        path = self.service.run_astar(mock_occupancy_grid, (0, 0))
        expected_path = [(0, 0), (0, 1), (0, 2), (1, 2), (2, 2)]

        self.assertEqual(path, expected_path)

    @patch.object(AStarService, "_AStarService__convert_occupancy_grid_to_grid")
    @patch.object(AStarService, "_AStarService__astar_search")
    def test_run_astar_integration(self, mock_astar_search, mock_convert_grid):
        mock_occupancy_grid = self.__occupancy_grid
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
