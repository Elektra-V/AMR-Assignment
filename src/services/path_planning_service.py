import math
from typing import List, Tuple

from nav_msgs.msg import OccupancyGrid, Odometry
from sensor_msgs.msg import LaserScan

from src.common.constants import Constants
from src.common.types import InputMessageFull
from src.services.astar_service import AStarService
from src.services.potential_field_service import PotentialFieldService
from src.utils.event_bus import EventBus

REPULSIVE_DISTANCE = 0.5


class PathPlanningService:
    __latest_map: OccupancyGrid | None = None
    __latest_odometry: Odometry | None = None
    __latest_scan: LaserScan | None = None

    def __init__(
        self,
        event_bus: EventBus,
        initial_position: Tuple[float, float],
        goal_position: Tuple[float, float],
    ) -> None:
        self.__event_bus = event_bus
        self.__current_position = initial_position
        self.__goal_position = goal_position

        self.__potential_field = PotentialFieldService()

    def run_path_planner(self, msg: InputMessageFull):
        if isinstance(msg, OccupancyGrid):
            self.__latest_map = msg
            self.__astar = AStarService(self.__latest_map.info.resolution)
        if isinstance(msg, Odometry):
            self.__latest_odometry = msg
        if isinstance(msg, LaserScan):
            self.__latest_scan = msg

        if (
            self.__latest_map is not None
            and self.__latest_odometry is not None
            and self.__latest_scan is not None
        ):
            path = self.__astar.run_astar(
                self.__obtain_obstacle_locations(),
                self.__current_position,
                self.__goal_position,
            )
            self.__current_position = path

            twist_message = self.__potential_field.run_potential_field(
                self.__current_position,
                (
                    int(self.__goal_position[0]),
                    int(self.__goal_position[1]),
                ),
                self.__latest_odometry,
                self.__latest_scan,
            )
            self.__event_bus.publish(Constants.EVENT_TOPIC_MOVEMENT, twist_message)

    def __obtain_obstacle_locations(self) -> List[Tuple[float, float]]:
        obstacles = []

        if self.__latest_scan is None:
            return []

        laser_scan = self.__latest_scan
        for i, range_value in enumerate(laser_scan.ranges):
            if 0 < range_value < REPULSIVE_DISTANCE:
                obstacle_angle = laser_scan.angle_min + i * laser_scan.angle_increment
                x_local = range_value * math.cos(obstacle_angle)
                y_local = range_value * math.sin(obstacle_angle)

                obstacles.append((x_local, y_local))

        return obstacles
