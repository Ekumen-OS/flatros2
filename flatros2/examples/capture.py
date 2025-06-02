import cv2
import numpy as np

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from sensor_msgs.msg import Image

from flatros2.message import Flat
from flatros2.publisher import FlatPublisher

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
    encoding="bgr8",
    step=1280 * 3,
    data=np.zeros(1280 * 720 * 3, dtype=np.uint8)
))

class ImageCapture(Node):

    def __init__(self) -> None:
        super().__init__("image_capture", start_parameter_services=False, enable_logger_service=False)
        #self.pub = FlatPublisher(self, VGAImage, "image", 10)
        self.pub = FlatPublisher(self, HDImage, "image", 10)

    def spin(self) -> None:
        capture = cv2.VideoCapture(0)
        #capture.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        #capture.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        capture.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
        capture.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
        while rclpy.ok():
            ret, frame = capture.read()
            if not ret:
                print("Failed to receive frame. Exiting ...")
                break
            message = self.pub.borrow_loaned_message()
            message.data = frame.tobytes()
            self.pub.publish_loaned_message(message)


def main(args=None):
    rclpy.init(args=args)
    try:
        node = ImageCapture()
        node.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        rclpy.try_shutdown()

if __name__ == '__main__':
    main()
