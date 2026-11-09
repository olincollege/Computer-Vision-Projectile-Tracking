
# Imports
import cv2
import rclpy
import math
import numpy as np

from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image #TODO: Add dependencies in setup.py

class BallDetector(Node):
    def __init__(self):
        super().__init__('ball_detector')
        self.get_logger().info('Ball Detector Node has been started.')

        # Establish Publishers and Subscribers
        """
        1. *Create a subscriber for the video camera feed
        2. *Create a subscriber for the camera calibration data
        3. Create a publisher for the ball position and radius
        """
        self.videos = self.create_subscription(Image, 'image_topic', self.image_callback, 10)
        
        # Handle subscriptions
        def image_callback(self, msg):
            pass
        # Ball Dectection
        def detect_ball(calibrated_images):
            detected_balls = []
            
            for image in calibrated_images: #For each calibrated image...
                photo = cv2.imread(image)   #...Load the image into OpenCV

                if photo is None:
                    self.get_logger().warn(f"Could not load image: {image}")
                    continue

                #Convert image to grayscale
                gray_image = cv2.cvtColor(photo, cv2.COLOR_BGR2GRAY)

                # Set minDist for Hough Circle Transform based on diagonal image size
                h, w = photo.shape[:2] #Get the height and width of the image
                minDist = math.hypot(h, w) + 1 #Set the minimum distance between detected circles

                # Apply Hough Circle Transform to detect circles in the image
                circles = cv2.HoughCircles(gray_image, cv2.HOUGH_GRADIENT_ALT, dp = 1.5, minDist = minDist, param1 = 300, param2 = 0.9, minRadius = 0, maxRadius = 0) #Detect circles in the image

                if circles is None:
                    self.get_logger().warn(f"No circles detected in image: {image}")
                    continue

                
                for x, y, radius in circles[0]:
                    # draw the circle
                    cv2.circle(photo, (x, y), radius, (0, 255, 0), thickness = 2)

                    detected_balls.append((x, y, radius))  # Add the detected ball position and radius to list

                # Display the detected circles on the image
                cv2.imshow('detected circles', photo)


            # Keep the window open and close when key pressed
            cv2.waitKey(0)
            cv2.destroyAllWindows()

            return tuple(detected_balls)  # Return the tuple of detected ball x, y, and radius

        # Package Publisher

# Main Run Loop
def main(args=None):
    """ Spin node. """
    rclpy.init()
    node = BallDetector()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()