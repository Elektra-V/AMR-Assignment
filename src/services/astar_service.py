from queue import PriorityQueue
from typing import List

from nav_msgs.msg import OccupancyGrid

from src.common.types import CoordinatesTuple, Grid


class AStarService:
    def __init__(self, goal_position: CoordinatesTuple):
        self.__goal_position = goal_position

    def run_astar(
        self, map: OccupancyGrid, start: CoordinatesTuple
    ) -> List[CoordinatesTuple]:
        grid = self.__convert_occupancy_grid_to_grid(map)
        return self.__astar_search(
            grid,
            start,
            self.__goal_position,
        )

    def __convert_occupancy_grid_to_grid(self, map: OccupancyGrid) -> Grid:
        width = map.info.width
        height = map.info.height

        grid: Grid = [[0 for _ in range(width)] for _ in range(height)]

        for i, value in enumerate(map.data):
            row: int = i // width
            column: int = i % width

            grid[row][column] = 1 if value == 100 else 0 if value == 0 else -1

        return grid

    def __astar_search(
        self, grid: Grid, start: CoordinatesTuple, goal: CoordinatesTuple
    ) -> List[CoordinatesTuple]:
        frontier = PriorityQueue()
        frontier.put((0, start))
        came_from = {}
        cost_so_far = {}
        came_from[start] = None
        cost_so_far[start] = 0

        while not frontier.empty():
            current = frontier.get()[1]

            if current == goal:
                break

            for next in self.__get_neighbors(grid, current):
                new_cost = cost_so_far[current] + 1
                if next not in cost_so_far or new_cost < cost_so_far[next]:
                    cost_so_far[next] = new_cost
                    priority = new_cost + self.__heuristic(goal, next)
                    frontier.put((priority, next))
                    came_from[next] = current

        return self.__reconstruct_path(came_from, start, goal)

    def __get_neighbors(
        self, grid: Grid, node: CoordinatesTuple
    ) -> List[CoordinatesTuple]:
        # Up, Right, Down, Left
        directions = [(0, 1), (1, 0), (0, -1), (-1, 0)]
        neighbors = []
        for direction in directions:
            neighbor = (node[0] + direction[0], node[1] + direction[1])
            if 0 <= neighbor[0] < len(grid) and 0 <= neighbor[1] < len(grid[0]):
                if grid[neighbor[0]][neighbor[1]] == 0:
                    neighbors.append(neighbor)
        return neighbors

    def __heuristic(self, a: CoordinatesTuple, b: CoordinatesTuple) -> float:
        # Manhattan distance heuristic
        return abs(a[0] - b[0]) + abs(a[1] - b[1])

    def __reconstruct_path(
        self, came_from: dict, start: CoordinatesTuple, goal: CoordinatesTuple
    ) -> List[CoordinatesTuple]:
        current = goal
        path = []
        while current != start:
            path.append(current)
            current = came_from[current]
        path.append(start)
        path.reverse()
        return path
