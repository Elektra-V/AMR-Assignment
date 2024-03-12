from queue import PriorityQueue
from typing import Dict, List, Tuple

from nav_msgs.msg import OccupancyGrid

from src.common.coordinate_converter import world_to_grid
from src.common.types import CoordinatesTuple, Grid


class AStarService:
    def run_astar(
        self,
        current_position: Tuple[float, float],
        goal_position: Tuple[float, float],
        map: OccupancyGrid,
    ) -> List[Tuple[float, float]]:
        start = world_to_grid(current_position[0], current_position[1], map)
        goal = world_to_grid(goal_position[0], goal_position[1], map)
        return self.__astar_search(start, goal, map)

    def __astar_search(
        self,
        start: CoordinatesTuple,
        goal: CoordinatesTuple,
        map: OccupancyGrid,
    ) -> List[Tuple[float, float]]:
        closed = set()
        fringe = PriorityQueue()
        fringe.put((self.__heuristic(start, goal), start, []))
        grid = self.__convert_occupancy_grid_to_grid(map)

        while not fringe.empty():
            _, current_state, path = fringe.get()
            if goal == current_state:
                return path

            closed.add(current_state)

            for next_state in self.__get_possible_moves(grid, current_state):
                if next_state in closed:
                    continue

                new_path = path + [next_state]
                f_cost = len(new_path) + self.__heuristic(next_state, goal)

                if any(
                    next_state == state and cost <= f_cost
                    for cost, state, _ in fringe.queue
                ):
                    continue

                fringe.put((f_cost, next_state, new_path))

        return []

    def __heuristic(self, a: CoordinatesTuple, b: CoordinatesTuple) -> float:
        return abs(a[0] - b[0]) + abs(a[1] - b[1])

    def __get_possible_moves(
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

    def __convert_occupancy_grid_to_grid(self, map: OccupancyGrid) -> Grid:
        width, height = map.info.width, map.info.height
        grid: Grid = [
            [0 if value < 60 else 1 for value in map.data[i * width : (i + 1) * width]]
            for i in range(height)
        ]
        return grid
