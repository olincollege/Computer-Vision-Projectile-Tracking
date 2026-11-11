# Imports
import rclpy
import math
import yaml
import numpy as np
import cv2 as cv
import glob
import matplotlib.pyplot as plt

from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from ament_index_python.packages import get_package_share_directory



class BallDetector(Node):
    def __init__(self):
        super().__init__('ball_detector')
        self.get_logger().info('Ball Detector Node has been started.')

        # Establish Publishers and Subscribers
        """
        1. *Create a subscriber for the video camera feed
        2. Create a publisher for the ball position and radius
        """
        self.ball_publisher = self.create_publisher(Float32MultiArray, "detection", 10)
        
        # Callibration Parameters
        # Find the directory of the yaml file
        share_dir = get_package_share_directory("projectile_tracking")
        yaml_path = share_dir + '/config/calibration.yaml'

        # Open it and read its contents
        with open(yaml_path, "r", encoding="utf-8") as file:
            yaml_dict = yaml.safe_load(file)

        # Unpack the data returned by cv.calibrateCamera()
        self.mtx = np.array(yaml_dict["mtx"])
        self.dist = np.array(yaml_dict["dist"])

        # Open and prepare the image
        self.uncalibrated_images = glob.glob(share_dir + "/photos/_Q1A8026.png")
        self.uncalibrated_video_frames = glob.glob(share_dir + "/video_photos/*.png")
        self.uncalibrated_z_photos = glob.glob(share_dir + "/z_photos/*.png")
        self.detection_timer = self.create_timer(1.0, self.run_detection_once)


    def load_images(self):
        """
        Load images from the specified directory.
        """
        loaded_images = []
            
        for image_path in self.uncalibrated_images:
            image = cv.imread(image_path)
            if image is None:
                self.get_logger().warn(f"Could not load image: {image_path}")
                continue
            loaded_images.append(image)
            
        return loaded_images

    def undistort_image(self, images):
        """
        Undistort the input image using the calibration parameters.
        """
        # Find the directory of the yaml file

        undistorted_images = []

        for image in images:
            if image is None:
                self.get_logger().warn(f"Could not load image: {image}")
                continue

        # Get image dimensions
            h,  w = image.shape[:2]
            newcameramtx, roi = cv.getOptimalNewCameraMatrix(self.mtx, self.dist, (w,h), 1, (w,h))

        # Undistort the image
            undistorted_image = cv.undistort(image, self.mtx, self.dist, None, newcameramtx)

        # Crop the image
            x, y, w, h = roi
            undistorted_image = undistorted_image[y:y+h, x:x+w]
            undistorted_images.append(undistorted_image)
        
        return undistorted_images
    
    def calibration(self):
        loaded_z_photos = self.load_images(self.uncalibrated_z_photos)
        calibrated_frames = self.undistort_image(loaded_z_photos)
        z_list = self.detect_ball(calibrated_frames)

        ball_radius = np.array([ball[2] for ball in z_list], dtype=float)
        distance_step = np.arange(4, 31, dtype=float)
    
        a, b = np.polyfit(ball_radius, distance_step, 1)

        # Save coefficients for future distance estimates.
        self.distance_a = a
        self.distance_b = b

        # Use an ascending, smooth radius range to draw the curve.
        radius_curve = np.linspace(
            ball_radius.min(),
            ball_radius.max(),
            200,
        )
        distance_curve = a * radius_curve + b

        plt.scatter(
            ball_radius,
            distance_step,
            label="Calibration measurements",
        )
        plt.plot(
            radius_curve,
            distance_curve,
            color="red",
            label="Inverse best fit",
        )

        plt.xlabel("Ball radius (pixels)")
        plt.ylabel("Camera-to-ball distance (ft)")
        plt.title("Ball Distance Calibration")
        plt.grid(True)
        plt.legend()
        plt.tight_layout()
        plt.show()

        return a, b

    def get_distance(self, radius):
        distance = self.distance_a * radius + self.distance_b
        return distance
    
    def detect_ball(self, calibrated_images):
        detected_balls = []
        scale_down = 0.13  
        
        for image in calibrated_images: #For each calibrated image...
            photo = cv.resize(image, None, fx=scale_down, fy=scale_down, interpolation=cv.INTER_LINEAR)

            # Set minimum and max HSV values for the ball
            share_dir = get_package_share_directory("projectile_tracking")
            yaml_path = share_dir + '/config/color_mask.yaml'
            with open(yaml_path, "r", encoding="utf-8") as file:
                color_range = yaml.safe_load(file)
            lower = np.array([color_range["hMin"], color_range["sMin"], color_range["vMin"]])
            upper = np.array([color_range["hMax"], color_range["sMax"], color_range["vMax"]])

            # Create HSV Image and threshold into a range.
            hsv = cv.cvtColor(photo, cv.COLOR_BGR2HSV)
            gray_image = cv.inRange(hsv, lower, upper)

            # Set minDist for Hough Circle Transform based on diagonal image size
            h, w = photo.shape[:2] #Get the height and width of the image
            min_dist = math.hypot(h, w) + 1 #Set the minimum distance between detected circles

            # Apply Hough Circle Transform to detect circles in the image
            circles = cv.HoughCircles(gray_image, cv.HOUGH_GRADIENT_ALT, dp = 1.5, minDist = min_dist, param1 = 300, param2 = 0.6, minRadius = 0, maxRadius = 0) #Detect circles in the image

            if circles is None:
                self.get_logger().info(f"No circles detected, skipping image {image}")
                continue
            
            for x, y, radius in circles[0]:
                # draw the circle
                cv.circle(photo, (int(x), int(y)), int(radius), (0, 255, 0), thickness = 2)
                distance = self.get_distance(radius)  # Call the get_distance function to calculate the distance based on the detected radius
                
                detected_balls.append((x, y, radius, w, h, distance))  # Add the detected ball position and radius to list


        return detected_balls  # Return list of detected ball x, y, radius, and image dimensions (width, height)


        # Package Publisher
    def publish_ball_data(self, ball_data):
        """
        Publish the ball position, radius, and image dimensions as a Float32MultiArray message.
        """
        msg = Float32MultiArray()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.data = [float(value) for value in ball_data]
        self.ball_publisher.publish(msg)

    def run_detection(self):
        self.get_logger().info('Running ball detection...')
        original_images = self.load_images()
        calibrated_images = self.undistort_image(original_images)
        detected_balls = self.detect_ball(calibrated_images)

        for ball_data in detected_balls:
            self.publish_ball_data(ball_data)

        self.get_logger().info(f"Published {len(detected_balls)} detections.")

    def run_detection_once(self):
        self.detection_timer.cancel()
        self.run_detection()

# Main Run Loop
def main(args=None):
    """ Spin node. """
    rclpy.init()
    node = BallDetector()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()