from typing import Tuple

from geometry_msgs.msg import Twist
from rclpy.node import Node

from src.common.constants import Constants
from src.common.types import TwistMessage
from src.utils.event_bus import EventBus


class SLAMPublisher(Node):
    def __init__(self, event_bus: EventBus) -> None:
        super().__init__(Constants.PUBLISHER_NODE_NAME)
        self.__event_bus = event_bus

        self.__publisher = self.create_publisher(
            msg_type=Twist, topic=Constants.ROS_TOPIC_MOVEMENT, qos_profile=10
        )
        self.__event_bus.subscribe(
            Constants.EVENT_TOPIC_MOVEMENT, self.__publish_velocity
        )

    def __publish_velocity(self, movement_input: TwistMessage) -> None:
        self._logger.info(f"Publishing command {movement_input}")
        msg = Twist()

        (linear_x, angular_z) = movement_input
        msg.linear.x = linear_x
        msg.angular.z = angular_z

        self.__publisher.publish(msg)
