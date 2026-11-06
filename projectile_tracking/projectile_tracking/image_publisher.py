""" Publish a video as set of images over time. """

import glob
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class ImagePublisher(Node):
    """ Publish a video as set of images over time. """
    def __init__(self):
        """ Take in all the images and initialize the Node. """
        super().__init__("image_publisher")

        # Take in the images.
        frame_paths = glob.glob("videos/*.png")
        self.frames = [cv2.imread(path) for path in frame_paths]
        self.frame = 0

        # Create the publisher.
        self.publisher = self.create_publisher(Image, "image_topic", 10)

        # Start the loop timer.
        self.timer = self.create_timer(0.25, self.publish_image)

        # Initialize the CvBridge
        self.bridge = CvBridge()

    def publish_image(self):
        """
        Publish the next image in self.frames
        """
        # Convert OpenCV image to ROS 2 Image message.
        image_msg = self.bridge.cv2_to_imgmsg(
            self.frames[self.frame], encoding="bgr8"
        )
        image_msg.header.stamp = self.get_clock().now().to_msg()
        image_msg.header.frame_id = "camera_frame"

        # Publish the image
        self.publisher.publish(image_msg)
        self.frame = (self.frame + 1) % len(self.frames)

def main(args=None):
    """ Spin the node. """
    rclpy.init()
    node = ImagePublisher()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
