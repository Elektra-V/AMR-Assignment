from typing import List, Tuple

import numpy as np
from nav_msgs.msg import OccupancyGrid


class FrontierService:
    def find_frontiers(
        self, occupancy_grid: OccupancyGrid
    ) -> List[Tuple[float, float]]:
        data = np.array(occupancy_grid.data).reshape(
            (occupancy_grid.info.height, occupancy_grid.info.width)
        )
        unknown_value = -1
        free_threshold = 50

        frontiers = []
        visited = np.zeros_like(data, dtype=bool)
        directions = [
            (dx, dy)
            for dx in [-1, 0, 1]
            for dy in [-1, 0, 1]
            if not (dx == 0 and dy == 0)
        ]

        for y in range(data.shape[0]):
            for x in range(data.shape[1]):
                if data[y, x] >= free_threshold or visited[y, x]:
                    continue

                if any(
                    data[
                        min(max(0, y + dy), data.shape[0] - 1),
                        min(max(0, x + dx), data.shape[1] - 1),
                    ]
                    == unknown_value
                    for dx, dy in directions
                ):
                    frontier_cells = self.__grow_frontier(
                        x, y, data, visited, free_threshold
                    )
                    if frontier_cells:
                        centroid_x = sum(x for x, _ in frontier_cells) / len(
                            frontier_cells
                        )
                        centroid_y = sum(y for _, y in frontier_cells) / len(
                            frontier_cells
                        )
                        frontiers.append((centroid_x, centroid_y))

        return frontiers

    def __grow_frontier(
        self,
        start_x: int,
        start_y: int,
        data: np.ndarray,
        visited: np.ndarray,
        free_threshold: int,
    ) -> List[Tuple[int, int]]:
        queue = [(start_x, start_y)]
        frontier = []

        while queue:
            x, y = queue.pop(0)
            if visited[y, x] or data[y, x] >= free_threshold:
                continue
            visited[y, x] = True
            frontier.append((x, y))

            for dx, dy in [(dx, dy) for dx in [-1, 0, 1] for dy in [-1, 0, 1]]:
                nx, ny = x + dx, y + dy
                if (
                    0 <= nx < data.shape[1]
                    and 0 <= ny < data.shape[0]
                    and not visited[ny, nx]
                    and data[ny, nx] < free_threshold
                ):
                    queue.append((nx, ny))

        return frontier
