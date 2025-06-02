from types import TracebackType
from typing import Callable, Generic, Optional, Type, TypeVar, Union

from rclpy.callback_groups import CallbackGroup
from rclpy.node import Node
from rclpy.qos import QoSProfile
from rclpy.waitable import NumberOfEntities, Waitable
from rclpy.impl.implementation_singleton import rclpy_implementation

from .flatros2_pybindings import FlatSubscriptionImpl

WaitSet = rclpy_implementation.WaitSet

MessageT = TypeVar("MessageT")

class FlatSubscription(Generic[MessageT], Waitable):

    def __init__(
        self,
        node: Node,
        topic_type: Type[MessageT],
        topic_name: str,
        callback: Callable[[MessageT], None],
        qos_profile: Union[int, QoSProfile],
        callback_group: Optional[CallbackGroup] = None,
    ):
        self.node = node
        self.topic_type = topic_type
        if callback_group is None:
            callback_group = self.node.default_callback_group
        super().__init__(callback_group)
        self.node._validate_topic_or_service_name(topic_name)
        topic_name = self.node.resolve_topic_name(topic_name)
        self.callback = callback
        if not isinstance(qos_profile, QoSProfile):
            qos_profile = QoSProfile(depth=qos_profile)
        self.qos_profile = qos_profile
        self.__sub = FlatSubscriptionImpl(
            self.node.handle, self.topic_type, topic_name,
            self.qos_profile.get_c_qos_profile()
        )
        self.node.add_waitable(self)

    @property
    def topic_name(self) -> str:
        with self.__sub:
            return self.__sub.get_topic_name()

    def destroy(self) -> None:
        self.node.remove_waitable(self)
        self.__sub.destroy_when_not_in_use()

    def __enter__(self) -> None:
        self.__sub.__enter__()

    def __exit__(
        self,
        exc_type: Optional[Type[BaseException]] = None,
        exc_val: Optional[BaseException] = None,
        exc_tb: Optional[TracebackType] = None,
    ) -> None:
        self.__sub.__exit__(exc_type, exc_val, exc_tb)

    def is_ready(self, wait_set: WaitSet) -> bool:
        with self.__sub, wait_set:
            return self.__sub.is_ready(wait_set)

    def take_data(self) -> MessageT:
        with self.__sub:
            return self.__sub.take_loaned_message()

    async def execute(self, taken_data: MessageT) -> None:
        """Execute work after data has been taken from a ready wait set."""
        with self.__sub:
            try:
                self.callback(taken_data)
            finally:
                return self.__sub.return_loaned_message(taken_data)

    def get_num_entities(self) -> NumberOfEntities:
        return NumberOfEntities(num_subs=1)

    def add_to_wait_set(self, wait_set: WaitSet) -> None:
        with self.__sub, wait_set:
            self.__sub.add_to_waitset(wait_set)
