import heapq
from typing import List, Tuple

from nav_msgs.msg import OccupancyGrid

from src.common.coordinate_converter import grid_to_world, world_to_grid
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

        path = self.__astar_search(start, goal, map)
        path_world = [grid_to_world(x, y, map) for x, y in path] if path else []
        return path_world

    def convert_occupancy_grid_to_grid(self, map: OccupancyGrid) -> Grid:
        width, height = map.info.width, map.info.height

        grid: Grid = [
            [1 if value >= 50 else 0 for value in map.data[i * width : (i + 1) * width]]
            for i in range(height)
        ]
        return grid

    def __astar_search(
        self,
        start: CoordinatesTuple,
        goal: CoordinatesTuple,
        map: OccupancyGrid,
    ):
        neighbors = [
            (0, 1),
            (0, -1),
            (1, 0),
            (-1, 0),
            (1, 1),
            (1, -1),
            (-1, 1),
            (-1, -1),
        ]
        closed = set()
        came_from = {}
        gscore = {start: 0.0}
        fscore = {start: self.__heuristic(start, goal)}
        oheap = []
        heapq.heappush(oheap, (fscore[start], start))

        while oheap:
            current = heapq.heappop(oheap)[1]

            if current == goal:
                return self.reconstruct_path(came_from, start, goal)

            closed.add(current)

            for i, j in neighbors:
                neighbor = current[0] + i, current[1] + j
                if not (
                    0 <= neighbor[0] < map.info.width
                    and 0 <= neighbor[1] < map.info.height
                ):
                    continue

                diagonal = abs(i) + abs(j) == 2
                tentative_g_score = gscore[current] + (
                    self.__heuristic(current, neighbor) + (1.4 if diagonal else 1.0)
                )

                if (
                    map.data[neighbor[1] * map.info.width + neighbor[0]] != 0
                    or neighbor in closed
                ):
                    continue

                if tentative_g_score < gscore.get(neighbor, float("inf")):
                    came_from[neighbor] = current
                    gscore[neighbor] = tentative_g_score
                    fscore[neighbor] = tentative_g_score + self.__heuristic(
                        neighbor, goal
                    )
                    if neighbor not in {n[1] for n in oheap}:
                        heapq.heappush(oheap, (fscore[neighbor], neighbor))

        return False

    def __heuristic(self, a: CoordinatesTuple, b: CoordinatesTuple) -> float:
        return abs(a[0] - b[0]) + abs(a[1] - b[1])

    def reconstruct_path(
        self, came_from: dict, start: CoordinatesTuple, goal: CoordinatesTuple
    ) -> List[Tuple[int, int]]:
        current = goal
        path = []
        while current in came_from and current != start:
            path.append(current)
            current = came_from[current]
        path.reverse()
        return path
