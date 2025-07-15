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
import cv_bridge
import numpy as np

import rclpy
import rclpy.time
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from sensor_msgs.msg import Image

from flatros2.subscription import FlatSubscription

class ImageViewerNode(Node):

    def __init__(self):
        super().__init__("image_viewer", start_parameter_services=False, enable_rosout=False, enable_logger_service=False)
        self.num_messages = 0
        self.start_time = self.get_clock().now()
        self.bridge = cv_bridge.CvBridge()
        self.sub = FlatSubscription(self, Image, "image", self.callback, 10)

    def callback(self, message):
        now = self.get_clock().now()
        message_stamp = rclpy.time.Time(
            seconds=message.header.stamp.sec,
            nanoseconds=message.header.stamp.nanosec, 
            clock_type=rclpy.time.ClockType.ROS_TIME)
        self.num_messages = self.num_messages + 1
        delay = now - message_stamp
        print("delay = ", delay.nanoseconds / 1e9)
        if self.num_messages > 10:
            elapsed_time = now - self.start_time
            print("freq = ", self.num_messages * 1e9 / elapsed_time.nanoseconds)
        cv2.imshow("gui", self.bridge.imgmsg_to_cv2(message))
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    try:
        rclpy.spin(ImageViewerNode())
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except Exception as e:
        input()
    finally:
        rclpy.try_shutdown()

if __name__ == '__main__':
    main()
