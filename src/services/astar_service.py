import heapq
from typing import Dict, List, Tuple


class AStarService:
    def __init__(self, grid_resolution: float):
        self.__grid_resolution = grid_resolution
        self.__motions = [
            (1, 0),
            (0, 1),
            (-1, 0),
            (0, -1),
            (1, 1),
            (-1, -1),
            (1, -1),
            (-1, 1),
        ]

    def run_astar(
        self,
        obstacles: List[Tuple[float, float]],
        current_position: Tuple[float, float],
        goal_position: Tuple[float, float],
    ) -> Tuple[int, int]:
        obstacles_after_resolution = []
        for obstacle in obstacles:
            obstacles_after_resolution.append(
                (
                    int(obstacle[0] / self.__grid_resolution),
                    int(obstacle[1] / self.__grid_resolution),
                )
            )
        current_position_after_resolution = (
            int(current_position[0] / self.__grid_resolution),
            int(current_position[1] / self.__grid_resolution),
        )
        goal_position_after_resolution = (
            int(goal_position[0] / self.__grid_resolution),
            int(goal_position[1] / self.__grid_resolution),
        )

        path = self.__astar_search(
            obstacles_after_resolution,
            current_position_after_resolution,
            goal_position_after_resolution,
        )
        return path[1] if len(path) > 1 else current_position_after_resolution

    def __astar_search(
        self,
        obstacles: List[Tuple[int, int]],
        start: Tuple[int, int],
        goal: Tuple[int, int],
    ) -> List[Tuple[int, int]]:
        open_set = []
        came_from = {}
        g_score = {start: 0}
        f_score = {start: self.__heuristic(start, goal)}

        heapq.heappush(open_set, (f_score[start], start))

        while open_set:
            _, current = heapq.heappop(open_set)

            if current == goal:
                return self.__reconstruct_path(came_from, current)

            for neighbor in self.__get_neighbors(obstacles, current):
                tentative_g_score = g_score[current]

                if neighbor not in g_score or tentative_g_score < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g_score
                    f_score[neighbor] = g_score[neighbor] + self.__heuristic(
                        neighbor, goal
                    )
                    if neighbor not in [i[1] for i in open_set]:
                        heapq.heappush(open_set, (f_score[neighbor], neighbor))

        return []

    def __get_neighbors(
        self, obstacles: List[Tuple[int, int]], node: Tuple[int, int]
    ) -> List[Tuple[int, int]]:
        neighbors = [
            (node[0] + motion[0], node[1] + motion[1]) for motion in self.__motions
        ]
        return [
            n
            for n in neighbors
            if n not in obstacles
            and 0 <= n[0] < self.__grid_resolution
            and 0 <= n[1] < self.__grid_resolution
        ]

    def __heuristic(self, node: Tuple[int, int], goal: Tuple[int, int]) -> int:
        # Manhattan distance for heuristic
        return abs(goal[0] - node[0]) + abs(goal[1] - node[1])

    def __reconstruct_path(
        self,
        came_from: Dict[Tuple[int, int], Tuple[int, int]],
        current: Tuple[int, int],
    ) -> List[Tuple[int, int]]:
        total_path = [current]
        while current in came_from:
            current = came_from[current]
            total_path.append(current)
        total_path.reverse()
        return total_path
