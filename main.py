import rclpy
from rclpy.executors import MultiThreadedExecutor

from src.publishers import SLAMPublisher
from src.services.path_planning_service import PathPlanningService
from src.subscribers import SLAMSubscriber
from src.utils.event_bus import EventBus


def main():
    rclpy.init()

    event_bus = EventBus()

    path_planning_service = PathPlanningService(
        event_bus=event_bus,
        initial_position=(-2.0, -3.5),
        goal_position=(3, -1.5),
    )

    subscriber = SLAMSubscriber(path_planning_service=path_planning_service)
    publisher = SLAMPublisher(event_bus=event_bus)
    executor = MultiThreadedExecutor()
    executor.add_node(subscriber)
    executor.add_node(publisher)

    try:
        executor.spin()
    finally:
        subscriber.destroy_node()
        publisher.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
