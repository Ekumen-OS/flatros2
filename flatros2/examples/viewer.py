
import cv2
import cv_bridge
import numpy as np

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from sensor_msgs.msg import Image

from flatros2.message import Flat
from flatros2.subscription import FlatSubscription

VGAImage = Flat(Image(
    width=640,
    height=480,
    encoding="bgr8",
    step=640 * 3,
    data=np.zeros(640 * 480 * 3, dtype=np.uint8)
))

HDImage = Flat(Image(
    width=1280,
    height=720,
    encoding="rgb8",
    step=1280 * 3,
    data=np.zeros(1280 * 720 * 3, dtype=np.uint8)
))

FHDImage = Flat(Image(
    width=1920,
    height=1280,
    encoding="bgr8",
    step=1920 * 3,
    data=np.zeros(1920 * 1280 * 3, dtype=np.uint8)
))


import time

class ImageViewer(Node):

    def __init__(self):
        super().__init__("image_viewer", start_parameter_services=False, enable_logger_service=False)
        self.bridge = cv_bridge.CvBridge()
        #self.sub = FlatSubscription(self, VGAImage, "image", self.callback, 10)
        self.sub = FlatSubscription(self, HDImage, "image", self.callback, 10)

    def callback(self, message):
        print(time.time(), self.bridge.imgmsg_to_cv2(message).shape)

def main(args=None):
    rclpy.init(args=args)
    try:
        node = ImageViewer()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        rclpy.try_shutdown()

if __name__ == '__main__':
    main()
