from dataclasses import dataclass
from typing import List, Tuple

from geometry_msgs.msg import Point, Quaternion
from nav_msgs.msg import OccupancyGrid, Odometry
from sensor_msgs.msg import LaserScan

InputMessageFull = OccupancyGrid | Odometry | LaserScan
Grid = List[List[int]]
CoordinatesTuple = Tuple[int, int]


@dataclass
class TwistMessage:
    linear_velocity_x: float
    angular_velocity_z: float


@dataclass
class Particle:
    position: Point
    orientation: Quaternion
    weight: float
