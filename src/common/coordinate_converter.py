from typing import Tuple

from nav_msgs.msg import OccupancyGrid

from src.common.types import CoordinatesTuple


def world_to_grid(
    x_world: float, y_world: float, occupancy_grid: OccupancyGrid
) -> CoordinatesTuple:
    resolution = occupancy_grid.info.resolution
    origin = occupancy_grid.info.origin

    x_relative = x_world - origin.position.x
    y_relative = y_world - origin.position.y

    x_grid = int(x_relative / resolution)
    y_grid = int(y_relative / resolution)

    return x_grid, y_grid


def grid_to_world(
    x_grid: int, y_grid: int, occupancy_grid: OccupancyGrid
) -> Tuple[float, float]:
    resolution = occupancy_grid.info.resolution
    origin = occupancy_grid.info.origin

    x_world = (x_grid * resolution) + origin.position.x
    y_world = (y_grid * resolution) + origin.position.y

    return x_world, y_world
