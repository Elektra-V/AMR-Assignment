import rclpy
from rclpy.executors import MultiThreadedExecutor

from src.publishers import SLAMPublisher
from src.services.astar_service import AStarService
from src.services.exploration_service import ExplorationService
from src.services.frontier_service import FrontierService
from src.services.localization_service import LocalizationService
from src.services.path_planning_service import PathPlanningService
from src.services.potential_field_service import PotentialFieldService
from src.subscribers import SLAMSubscriber
from src.utils.event_bus import EventBus


def main():
    rclpy.init()

    event_bus = EventBus()

    astar_service = AStarService()
    potential_field_service = PotentialFieldService()
    frontier_service = FrontierService()

    path_planning_service = PathPlanningService(
        event_bus=event_bus,
        astar_service=astar_service,
        potential_field_service=potential_field_service,
        initial_position=(0, -3.5),
        goal_position=(6.0, 0),
    )
    localization_service = LocalizationService(
        event_bus=event_bus, initial_position=(0.0, 0.0)
    )
    exploration_service = ExplorationService(
        event_bus=event_bus,
        astar_service=astar_service,
        potential_field_service=potential_field_service,
        frontier_service=frontier_service,
    )

    subscriber = SLAMSubscriber(
        path_planning_service=path_planning_service,
        localization_service=localization_service,
        exploration_service=exploration_service,
    )
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
