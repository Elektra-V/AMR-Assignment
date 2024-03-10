from typing import Any, Callable, Dict, List


class EventBus:
    def __init__(self) -> None:
        self.__listeners: Dict[str, List[Callable[[Any], None]]] = {}

    def subscribe(self, topic: str, listener: Callable[[Any], None]) -> None:
        if topic not in self.__listeners:
            self.__listeners[topic] = []

        self.__listeners[topic].append(listener)

    def publish(self, topic: str, data: Any) -> None:
        if topic in self.__listeners:
            for listener in self.__listeners[topic]:
                listener(data)
