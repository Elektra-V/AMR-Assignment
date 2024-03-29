from typing import Tuple

from nav_msgs.msg import OccupancyGrid, Odometry
from sensor_msgs.msg import LaserScan

from src.common.constants import Constants
from src.common.types import InputMessageFull
from src.services.astar_service import AStarService
from src.services.potential_field_service import PotentialFieldService
from src.utils.event_bus import EventBus


class PathPlanningService:
    __latest_map: OccupancyGrid | None = None
    __latest_odometry: Odometry | None = None
    __latest_scan: LaserScan | None = None

    def __init__(
        self,
        event_bus: EventBus,
        astar_service: AStarService,
        potential_field_service: PotentialFieldService,
        initial_position: Tuple[float, float],
        goal_position: Tuple[float, float],
    ) -> None:
        self.__event_bus = event_bus
        self.__current_position = initial_position
        self.__goal_position = goal_position

        self.__astar_service = astar_service
        self.__potential_field_service = potential_field_service

    def run_path_planner(self, msg: InputMessageFull) -> None:
        if isinstance(msg, OccupancyGrid):
            self.__latest_map = msg
        if isinstance(msg, Odometry):
            self.__latest_odometry = msg
        if isinstance(msg, LaserScan):
            self.__latest_scan = msg

        if (
            self.__latest_map is not None
            and self.__latest_odometry is not None
            and self.__latest_scan is not None
        ):
            path = self.__astar_service.run_astar(
                self.__current_position,
                self.__goal_position,
                self.__latest_map,
            )
            print(path)
            if len(path) >= 2:
                self.__current_position = path[1]

                twist_message = self.__potential_field_service.run_potential_field(
                    self.__current_position,
                    self.__goal_position,
                    self.__latest_odometry,
                    self.__latest_scan,
                )
                self.__event_bus.publish(Constants.EVENT_TOPIC_MOVEMENT, twist_message)
