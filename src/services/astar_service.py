import heapq
from queue import PriorityQueue
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
            [
                1 if value == 100 else 0
                for value in map.data[i * width : (i + 1) * width]
            ]
            for i in range(height)
        ]
        return grid

    def __astar_search(
        self,
        start: CoordinatesTuple,
        goal: CoordinatesTuple,
        map: OccupancyGrid,
    ) -> List[Tuple[int, int]]:
        neighbors = [(0, 1), (0, -1), (1, 0), (-1, 0), (1, 1), (1, -1), (-1, 1), (-1, -1)]
        closed = set()
        came_from = {}
        gscore = {start: 0}
        fscore = {start: self.__heuristic(start, goal)}
        oheap = []

        heapq.heappush(oheap, (fscore[start], start))

        while oheap:
            current = heapq.heappop(oheap)[1]
            
            if current == goal:
                data = []
                while current in came_from:
                    data.append(current)
                    current = came_from[current]
                data.append(start)
                return data[::-1]
            
            closed.add(current)

            for i, j in neighbors:
                neighbor = current[0] + i, current[1] + j
                tentative_g_score = gscore[current] + self.heuristic(current, neighbor)
                
                if 0 <= neighbor[0] < len(map) and 0 <= neighbor[1] < len(map[0]) and map[neighbor[0]][neighbor[1]] == 0:
                    if neighbor in closed and tentative_g_score >= gscore.get(neighbor, float('inf')):
                        continue
                    if tentative_g_score < gscore.get(neighbor, float('inf')) or neighbor not in [i[1] for i in oheap]:
                        came_from[neighbor] = current
                        gscore[neighbor] = tentative_g_score
                        fscore[neighbor] = tentative_g_score + self.heuristic(neighbor, goal)
                        heapq.heappush(oheap, (fscore[neighbor], neighbor))
                else:
                    continue
        return False

    def __heuristic(self, a: CoordinatesTuple, b: CoordinatesTuple) -> float:
        return abs(a[0] - b[0]) + abs(a[1] - b[1])