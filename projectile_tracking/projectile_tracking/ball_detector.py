"""Detect balls in calibrated images and publish detections through ROS 2.

The node loads camera calibration and HSV color thresholds from YAML files,
corrects lens distortion, detects circles in color-filtered images, and
publishes each detection as [x, y, radius, image_width, image_height].
"""

# Standard-library imports
import math
import glob
import rclpy
import yaml
import numpy as np
import cv2 as cv
import matplotlib.pyplot as plt

# ROS 2 interfaces and package utilities
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from ament_index_python.packages import get_package_share_directory


class BallDetector(Node):
    """Run a one-shot, image-based ball detection pipeline.

    Camera calibration and color thresholds are loaded from the
    ``projectile_tracking`` package. Detections are published on the
    ``detection`` topic as Float32MultiArray messages.

    Detection coordinates, radii, and image dimensions refer to the resized,
    undistorted, cropped images used during circle detection.
    """

    def __init__(self):
        """Initialize the publisher, calibration data, image paths, and timer."""
        super().__init__('ball_detector')
        self.get_logger().info('Ball Detector Node has been started.')

        # Set up publisher for ball x and y center, radius, and image dimensions
        self.ball_publisher = self.create_publisher(
            Float32MultiArray, "detection", 10
        )

        # Locate the camera calibration file in the package's share directory
        share_dir = get_package_share_directory("projectile_tracking")
        image_yaml_path = share_dir + '/config/calibration.yaml'

        # Read camera calibration parameters from YAML
        with open(image_yaml_path, "r", encoding="utf-8") as image_file:
            image_yaml_dict = yaml.safe_load(image_file)

        # Convert the camera matrix and distortion coefficients into arrays
        self.mtx = np.array(image_yaml_dict["mtx"])
        self.dist = np.array(image_yaml_dict["dist"])

        # Collect paths for the image sets
        frame_paths = glob.glob(share_dir + "/photos/*.png")
        self.uncalibrated_images = frame_paths[:1]
        print(self.uncalibrated_images)


        # Read mask parameters from YAML
        mask_yaml_path = share_dir + '/config/color_mask.yaml'

        with open(mask_yaml_path, "r", encoding="utf-8") as mask_file:
            color_range = yaml.safe_load(mask_file)

        self.lower = np.array([
                color_range["hMin"],
                color_range["sMin"],
                color_range["vMin"],
            ])
        self.upper = np.array([
                color_range["hMax"],
                color_range["sMax"],
                color_range["vMax"],
            ])

        # Schedule detection after one second. The callback cancels this timer
        # so that detection is performed only once
        self.detection_timer = self.create_timer(
            1.0, self.run_detection_once
        )

    def load_images(self):
        """Read the configured still images from disk.

        Unreadable images are logged and skipped.

        Returns:
            list[numpy.ndarray]: Successfully loaded images in BGR format.
        """
        loaded_images = []

        for image_path in self.uncalibrated_images:
            image = cv.imread(image_path)

            # Avoid passing failed image loads to subsequent processing stages
            if image is None:
                self.get_logger().warn(
                    f"Could not load image: {image_path}"
                )
                continue

            loaded_images.append(image)
        return loaded_images

    def undistort_image(self, images):
        """Correct lens distortion and crop each image to its valid region.

        Args:
            images (list[numpy.ndarray]): Images to undistort.

        Returns:
            list[numpy.ndarray]: Undistorted images cropped to the region
            returned by the camera-matrix calculation.
        """
        undistorted_images = []

        for image in images:
            # Skip missing images if this method is called independently
            # of load_images().
            if image is None:
                self.get_logger().warn(
                    f"Could not load image: {image}"
                )
                continue

            # Calculate an adjusted camera matrix for this image's dimensions
            h, w = image.shape[:2]
            newcameramtx, roi = cv.getOptimalNewCameraMatrix(
                self.mtx, self.dist, (w, h), 1, (w, h)
            )

            # Remove lens distortion using the stored calibration parameters
            undistorted_image = cv.undistort(
                image, self.mtx, self.dist, None, newcameramtx
            )

            # Crop to the region of interest returned by OpenCV
            x, y, w, h = roi
            undistorted_image = undistorted_image[y:y + h, x:x + w]
            undistorted_images.append(undistorted_image)

        return undistorted_images

    def detect_ball(self, calibrated_images):
        """Find ball candidates using an HSV mask and circle detection.

        Each input image is resized to 13% of its original dimensions before
        applying color thresholding and the Hough Circle Transform.

        Args:
            calibrated_images (list[numpy.ndarray]): Undistorted BGR images.

        Returns:
            list[tuple]: Detections represented as
            (x, y, radius, image_width, image_height). All values describe
            the resized image; coordinates and radii are not scaled back
            to the input image dimensions.
        """
        detected_balls = []
        scale_down = 0.13

        for image in calibrated_images:
            # Reduce image dimensions before masking and circle detection
            photo = cv.resize(
                image,
                None,
                fx=scale_down,
                fy=scale_down,
                interpolation=cv.INTER_LINEAR,
            )

            # Load the HSV thresholds defining the target ball's color.


            # Convert BGR to HSV, then produce a binary mask of pixels
            # whose color falls within the configured bounds
            hsv = cv.cvtColor(photo, cv.COLOR_BGR2HSV)
            color_mask = cv.inRange(hsv, self.lower, self.upper)
            masked_photo = cv.bitwise_and(photo, photo, mask=color_mask)

            # Set the required separation between circle centers larger than
            # the image diagonal to suppress multiple detections per image
            h, w = photo.shape[:2]
            min_dist = math.hypot(h, w) + 1

            gray_image = cv.cvtColor(masked_photo, cv.COLOR_BGR2GRAY)
            # Search the color mask for circles using the alternate
            # gradient-based Hough method
            circles = cv.HoughCircles(
                gray_image,
                cv.HOUGH_GRADIENT_ALT,
                dp=1.5,
                minDist=min_dist,
                param1=300,
                param2=0.6,
                minRadius=0,
                maxRadius=0,
            )

            # Skip images for which no circle candidates were returned
            if circles is None:
                self.get_logger().info(
                    f"No circles detected, skipping image {image}"
                )
                continue

            for x, y, radius in circles[0]:

                # Store geometry together with the processed image dimensions
                detected_balls.append((x, y, radius, w, h))

        return detected_balls

    def publish_ball_data(self, ball_data):
        """Publish one detection on the detection topic.

        Args:
            ball_data (tuple): Ball center, radius, and processed image
                dimensions in the order (x, y, radius, width, height).
        """
        msg = Float32MultiArray()

        #msg.header.stamp = self.get_clock().now().to_msg()

        # Convert NumPy scalar values to Python floats
        msg.data = [float(value) for value in ball_data]
        self.ball_publisher.publish(msg)

    def run_detection(self):
        """Load, undistort, and analyze images, then publish each detection."""
        self.get_logger().info('Running ball detection...')

        # Execute the processing stages in order
        original_images = self.load_images()
        calibrated_images = self.undistort_image(original_images)
        detected_balls = self.detect_ball(calibrated_images)

        # Publish a separate message for each detected ball
        for ball_data in detected_balls:
            self.publish_ball_data(ball_data)

        self.get_logger().info(
            f"Published {len(detected_balls)} detections."
        )

    def run_detection_once(self):
        """Cancel the recurring timer and execute one detection pass."""
        self.detection_timer.cancel()
        self.run_detection()


def main(args=None):
    """Initialize ROS 2, spin the ball detector node, and shut down.

    Args:
        args (list[str] | None): Reserved command-line arguments. The current
            implementation does not forward this parameter to rclpy.init().
    """
    rclpy.init()
    node = BallDetector()

    # Process callbacks, including the timer that starts detection
    rclpy.spin(node)

    # Shut down ROS 2 after spinning stops
    rclpy.shutdown()


if __name__ == '__main__':
    main()

#Unused code (included to demontrate thought process)
'''  
    # Create a calibration curve for the z axis based on the ball radius in each image
    def z_calibration(self):
        loaded_z_photos = self.load_images(self.uncalibrated_z_photos)
        calibrated_frames = self.undistort_image(loaded_z_photos)
        z_list = self.detect_ball(calibrated_frames)

        ball_radius = np.array([ball[2] for ball in z_list], dtype=float)
        distance_step = np.arange(4, 31, dtype=float)
    
        a, b = np.polyfit(1/ball_radius, distance_step, 1)

        # Save coefficients for future distance estimates.
        self.distance_a = a
        self.distance_b = b

        # Use an ascending, smooth radius range to draw the curve.
        radius_curve = np.linspace(
            ball_radius.min(),
            ball_radius.max(),
            200,
        )
        distance_curve = a / radius_curve + b

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

    # Calculate distance between ball and camera (z-axis) based on calibration curve
    def get_distance(self, radius):
        distance = self.distance_a * radius + self.distance_b
        return distance 
    '''