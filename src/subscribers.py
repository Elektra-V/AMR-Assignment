import os

from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

from src.common.constants import Constants, Env
from src.services.exploration_service import ExplorationService
from src.services.localization_service import LocalizationService
from src.services.path_planning_service import PathPlanningService


class SLAMSubscriber(Node):
    def __init__(
        self,
        path_planning_service: PathPlanningService,
        localization_service: LocalizationService,
        exploration_service: ExplorationService,
    ) -> None:
        super().__init__(Constants.SUBSCRIBER_NODE_NAME)
        self.__path_planning_service = path_planning_service
        self.__localization_service = localization_service
        self.__exploration_service = exploration_service
        self.__is_in_exploration = os.environ[Env.EXPLORATION_ENV].lower() == "true"

        self.__map_subscription = self.create_subscription(
            msg_type=OccupancyGrid,
            topic=Constants.ROS_TOPIC_MAP,
            callback=self.__map_subscription_callback,
            qos_profile=10,
        )
        self.__odom_subscription = self.create_subscription(
            msg_type=Odometry,
            topic=Constants.ROS_TOPIC_ODOM,
            callback=self.__odometry_subscription_callback,
            qos_profile=10,
        )
        self.__lidar_subscription = self.create_subscription(
            msg_type=LaserScan,
            topic=Constants.ROS_TOPIC_LIDAR,
            callback=self.__lidar_subscription_callback,
            qos_profile=10,
        )

    def __del__(self) -> None:
        self.__map_subscription.destroy()
        self.__odom_subscription.destroy()
        self.__lidar_subscription.destroy()

    def __map_subscription_callback(self, msg: OccupancyGrid) -> None:
        self.get_logger().info("Received a map message")
        self.get_logger().info(f"Map size: {len(msg.data)} cells")

        self.__path_planning_service.run_path_planner(msg)

    def __odometry_subscription_callback(self, msg: Odometry) -> None:
        if self.__is_in_exploration:
            self.__localization_service.update_odometry_and_scan(msg)
            self.__exploration_service.run_exploration_service(msg)
        else:
            self.__path_planning_service.run_path_planner(msg)

    def __lidar_subscription_callback(self, msg: LaserScan) -> None:
        if self.__is_in_exploration:
            self.__localization_service.update_odometry_and_scan(msg)
            self.__exploration_service.run_exploration_service(msg)
        else:
            self.__path_planning_service.run_path_planner(msg)
