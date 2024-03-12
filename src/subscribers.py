from os import getcwd, path

from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

from src.common.constants import Constants
from src.services.path_planning_service import PathPlanningService


class SLAMSubscriber(Node):
    def __init__(self, path_planning_service: PathPlanningService) -> None:
        super().__init__(Constants.SUBSCRIBER_NODE_NAME)
        self.__path_planning_service = path_planning_service

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

        # WARNING: this is for debugging/testing purposes only
        with open(path.join(getcwd(), "maps", "occupancy-grid.txt"), "a") as file:
            file.write(f"{msg.info.width}\n")
            file.write(f"{msg.info.height}\n")
            file.write(f"{msg.info.resolution}\n")
            file.write(f"{msg.info.origin.position.x}\n")
            file.write(f"{msg.info.origin.position.y}\n")
            file.write(f"{msg.data}")

        self.__path_planning_service.run_path_planner(msg)

    def __odometry_subscription_callback(self, msg: Odometry) -> None:
        self.__path_planning_service.run_path_planner(msg)

    def __lidar_subscription_callback(self, msg: LaserScan) -> None:
        self.__path_planning_service.run_path_planner(msg)
