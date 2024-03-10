import rclpy

from src.common.types import Coordinates
from src.publishers import SLAMPublisher
from src.services.path_planning_service import PathPlanningService
from src.subscribers import SLAMSubscriber
from src.utils.event_bus import EventBus


def main():
    rclpy.init()

    event_bus = EventBus()

    path_planning_service = PathPlanningService(
        event_bus=event_bus,
        initial_position=Coordinates(0, 0),
        goal_position=Coordinates(10, 4),
    )

    subscriber = SLAMSubscriber(path_planning_service=path_planning_service)
    publisher = SLAMPublisher(event_bus=event_bus)
    rclpy.spin(subscriber)
    rclpy.spin(publisher)

    subscriber.destroy_node()
    publisher.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
