# Copyright 2025 Ekumen, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from typing import Generic, Type, TypeVar, Union

from rclpy.node import Node
from rclpy.qos import QoSProfile
from rclpy.type_support import check_for_type_support

from flatros2.message import Flat
from .flatros2_pybindings import FlatPublisherImpl

MessageT = TypeVar("MessageT")

class FlatPublisher(Generic[MessageT]):

    def __init__(self, node: Node, topic_type: Type[MessageT], topic_name: str, qos_profile: Union[int, QoSProfile]) -> None:
        self.node = node
        if not isinstance(topic_type, Flat):
            topic_type = Flat(topic_type)
        self.topic_type = topic_type
        self.node._validate_topic_or_service_name(topic_name)
        topic_name = self.node.resolve_topic_name(topic_name)
        if not isinstance(qos_profile, QoSProfile):
            qos_profile = QoSProfile(depth=qos_profile)
        self.__pub = FlatPublisherImpl(node.handle, topic_type, topic_name, qos_profile.get_c_qos_profile())

    @property
    def topic_name(self) -> str:
        with self.__pub:
            return self.__pub.get_topic_name()

    def destroy(self) -> None:
        with self.__pub:
            self.__pub.destroy_when_not_in_use()

    def borrow_loaned_message(self) -> MessageT:
        with self.__pub:
            return self.__pub.borrow_loaned_message()

    def publish_loaned_message(self, message: MessageT) -> None:
        with self.__pub:
            self.__pub.publish_loaned_message(message)

    def return_loaned_message(self, message: MessageT) -> None:
        with self.__pub:
            self.__pub.return_loaned_message(message)
