import unittest
from unittest.mock import MagicMock

from src.utils.event_bus import EventBus


class EventBusTest(unittest.TestCase):
    def setUp(self) -> None:
        self.event_bus = EventBus()

    def test_single_subscription(self) -> None:
        listener = MagicMock()

        topic = "test_topic"
        self.event_bus.subscribe(topic, listener)

        data = {"key": "value"}
        self.event_bus.publish(topic, data)

        listener.assert_called_once_with(data)

    def test_publish_to_unsubscribed_topic(self) -> None:
        listener = MagicMock()

        subscribed_topic = "subscribed_topic"
        self.event_bus.subscribe(subscribed_topic, listener)

        different_topic = "different_topic"
        data = {"key": "value"}
        self.event_bus.publish(different_topic, data)

        listener.assert_not_called()

    def test_multiple_subscribers_same_topic(self) -> None:
        listener1 = MagicMock()
        listener2 = MagicMock()

        topic = "shared_topic"
        self.event_bus.subscribe(topic, listener1)
        self.event_bus.subscribe(topic, listener2)

        data = {"shared": "data"}
        self.event_bus.publish(topic, data)

        listener1.assert_called_once_with(data)
        listener2.assert_called_once_with(data)


if __name__ == "__main__":
    unittest.main()
