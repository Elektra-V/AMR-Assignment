from dataclasses import dataclass
from typing import List, Tuple

from nav_msgs.msg import OccupancyGrid, Odometry
from sensor_msgs.msg import LaserScan

InputMessageFull = OccupancyGrid | Odometry | LaserScan
Grid = List[List[int]]
CoordinatesTuple = Tuple[int | float, int | float]


@dataclass(frozen=True)
class Coordinates:
    x: int | float
    y: int | float


@dataclass
class TwistMessage:
    linear_velocity_x: float
    angular_velocity_z: float
