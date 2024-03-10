from queue import PriorityQueue
from typing import Dict, List, Tuple

from nav_msgs.msg import OccupancyGrid

from src.common.types import CoordinatesTuple, Grid


class AStarService:
    def __init__(
        self,
        goal_position_world: Tuple[float, float],
        origin: Tuple[float, float],
        resolution: float,
    ):
        self.goal_position_world = goal_position_world
        self.origin = origin
        self.resolution = resolution
        self.goal_position = self.__world_to_grid(goal_position_world)

    def run_astar(
        self, map: OccupancyGrid, start_world: Tuple[float, float]
    ) -> List[CoordinatesTuple]:
        start = self.__world_to_grid(start_world)
        grid = self.__convert_occupancy_grid_to_grid(map)
        return self.__astar_search(grid, start, self.goal_position)

    def __world_to_grid(
        self, world_coordinates: Tuple[float, float]
    ) -> CoordinatesTuple:
        grid_x = int((world_coordinates[0] - self.origin[0]) / self.resolution)
        grid_y = int((world_coordinates[1] - self.origin[1]) / self.resolution)
        return grid_x, grid_y

    def __convert_occupancy_grid_to_grid(self, map: OccupancyGrid) -> Grid:
        width, height = map.info.width, map.info.height
        grid: Grid = [
            [0 if value == 0 else 1 for value in map.data[i * width : (i + 1) * width]]
            for i in range(height)
        ]
        return grid

    def __astar_search(
        self, grid: Grid, start: CoordinatesTuple, goal: CoordinatesTuple
    ) -> List[CoordinatesTuple]:
        frontier = PriorityQueue()
        frontier.put((0, start))
        came_from: Dict[CoordinatesTuple, CoordinatesTuple] = {start: None}
        cost_so_far: Dict[CoordinatesTuple, int] = {start: 0}

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
        directions = [(0, 1), (1, 0), (0, -1), (-1, 0)]
        neighbors = [
            (node[0] + dx, node[1] + dy)
            for dx, dy in directions
            if 0 <= node[0] + dx < len(grid)
            and 0 <= node[1] + dy < len(grid[0])
            and grid[node[0] + dx][node[1] + dy] == 0
        ]
        return neighbors

    def __heuristic(self, a: CoordinatesTuple, b: CoordinatesTuple) -> float:
        return abs(a[0] - b[0]) + abs(a[1] - b[1])

    def __reconstruct_path(
        self,
        came_from: Dict[CoordinatesTuple, CoordinatesTuple],
        start: CoordinatesTuple,
        goal: CoordinatesTuple,
    ) -> List[CoordinatesTuple]:
        current = goal
        path = [current]
        while current != start:
            current = came_from[current]
            path.append(current)
        path.reverse()
        return path
