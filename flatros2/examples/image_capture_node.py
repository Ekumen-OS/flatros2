#! /usr/bin/env python3

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

import cv2
import numpy as np

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from builtin_interfaces.msg import Time
from sensor_msgs.msg import Image
from std_msgs.msg import Header

from flatros2.message import Flat
from flatros2.publisher import FlatPublisher

HDImage = Flat(Image(
    width=1280,
    height=720,
    encoding="bgr8",
    step=1280 * 3,
    data=np.zeros(1280 * 720 * 3, dtype=np.uint8)
))

class ImageCaptureNode(Node):

    def __init__(self) -> None:
        super().__init__("image_capture", start_parameter_services=False, enable_rosout=False, enable_logger_service=False)
        self.pub = FlatPublisher(self, HDImage, "image", 10)

    def spin(self) -> None:
        capture = cv2.VideoCapture(0)
        capture.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
        capture.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
        capture.set(cv2.CAP_PROP_FPS, 10)
        while rclpy.ok():
            ret, frame = capture.read()
            if not ret:
                print("Failed to receive frame. Exiting ...")
                break
            message = self.pub.borrow_loaned_message()
            stamp = self.get_clock().now().to_msg()
            message.header.stamp.sec = stamp.sec
            message.header.stamp.nanosec = stamp.nanosec
            message.data = frame.tobytes()
            self.pub.publish_loaned_message(message)

def main(args=None):
    rclpy.init(args=args)
    try:
        ImageCaptureNode().spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        rclpy.try_shutdown()

if __name__ == '__main__':
    main()
