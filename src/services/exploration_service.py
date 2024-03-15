from typing import List, Tuple

import numpy as np
from nav_msgs.msg import OccupancyGrid, Odometry
from sensor_msgs.msg import LaserScan

from src.common.constants import Constants
from src.common.types import InputMessageFull
from src.services.astar_service import AStarService
from src.services.frontier_service import FrontierService
from src.services.potential_field_service import PotentialFieldService
from src.utils.event_bus import EventBus


class ExplorationService:
    __latest_map: OccupancyGrid | None = None
    __latest_odometry: Odometry | None = None
    __latest_scan: LaserScan | None = None
    __current_position: Tuple[float, float] | None = None

    def __init__(
        self,
        event_bus: EventBus,
        astar_service: AStarService,
        potential_field_service: PotentialFieldService,
        frontier_service: FrontierService,
    ) -> None:
        self.__event_bus = event_bus
        self.__goal_position = (0.0, 0.0)

        self.__astar_service = astar_service
        self.__potential_field_service = potential_field_service
        self.__frontier_search_service = frontier_service

        self.__event_bus.subscribe(
            Constants.EVENT_TOPIC_LOCALIZED_LOCATION,
            self.__update_estimated_current_location,
        )

    def run_exploration_service(self, msg: InputMessageFull) -> None:
        if isinstance(msg, OccupancyGrid):
            self.__latest_map = msg
            self.__run_exploration()
        if isinstance(msg, Odometry):
            self.__latest_odometry = msg
        if isinstance(msg, LaserScan):
            self.__latest_scan = msg

    def __run_exploration(self) -> None:
        if (
            self.__latest_map is not None
            and self.__latest_odometry is not None
            and self.__latest_scan is not None
            and self.__current_position is not None
        ):
            frontiers = self.__frontier_search_service.find_frontiers(self.__latest_map)
            if frontiers:
                self.__goal_position = self.__select_next_goal(frontiers)
                if self.__goal_position is None:
                    return

                path = self.__astar_service.run_astar(
                    self.__current_position, self.__goal_position, self.__latest_map
                )
                if path and len(path) > 1:
                    next_position = path[1]
                    self.__current_position = next_position

                    twist_message = self.__potential_field_service.run_potential_field(
                        self.__current_position,
                        self.__goal_position,
                        self.__latest_odometry,
                        self.__latest_scan,
                    )
                    self.__event_bus.publish(
                        Constants.EVENT_TOPIC_MOVEMENT, twist_message
                    )

    def __select_next_goal(
        self, frontiers: List[Tuple[float, float]]
    ) -> Tuple[float, float] | None:
        if not frontiers:
            return None

        current_position = np.array(self.__current_position)
        return min(
            frontiers,
            key=lambda frontier: np.linalg.norm(
                current_position - np.array(frontier[0])
            ),
        )

    def __update_estimated_current_location(self, position: Tuple[float, float]):
        self.__current_position = position
