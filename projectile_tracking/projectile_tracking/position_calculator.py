"""
Calculate the 3D position of a ball based on its 2D projected position and radius.
"""

from geometry_msgs.msg import Point, PointStamped, Header
import rclpy
from rclpy.node import Node
import yaml
from ament_index_python.packages import get_package_share_directory

class PositionCalculator(Node):
    def __init__(self):
        """ Initialize the Node. """
        super().__init__("position_calculator")

        # Take in the parameters.
        # Find the directory of the yaml file
        share_dir = get_package_share_directory("projectile_tracking")
        yaml_path = share_dir + '/config/pos_calc_params.yaml'
        with open(yaml_path, "r", encoding="utf-8") as file:
            yaml_dict = yaml.safe_load(file)
        self.r_to_z = yaml_dict["r_to_z"]
        self.width_over_focal_length = yaml_dict["width_over_focal_length"]

        # Create the subscriber.
        self.create_subscription(
            tuple, "detection", self.calculate_position, 10)

        # Create the publisher.
        self.publisher = self.create_publisher(PointStamped, "ball_pos", 10)

    def calculate_position(self, msg):
        """
        Calculate the position of the ball and publish it.

        msg (tuple[5]<float>):
            The x position of the ball in the image in pixels.
            The y position of the ball in the image in pixels.
            The radius position of the ball in the image in pixels.
            The width of the image in pixels.
            The height of the image in pixels.
        """
        # Find the 2D x & y positions and radius
        w = msg[3]
        h = msg[4]
        x2d = msg[0] - 0.5*w
        y2d = -msg[1] - 0.5*h
        r2d = msg[2]

        # Calculate the z position of the ball using its radius.
        z3d = self.r_to_z / r2d

        # Calculate the x & y positions in 3D space.
        # 3D position =
        # 2d_position_(px) * z_3D_position (feet) * sensor_width (mm) /
        # focal_length / sensor_width (px)
        x3d = x2d * z3d * self.width_over_focal_length / w
        y3d = y2d * z3d * self.width_over_focal_length / w

        # Publish the 3D position.
        ball_pos = PointStamped()
        ball_pos.point.x = x3d
        ball_pos.point.y = y3d
        ball_pos.point.z = z3d
        ball_pos.header = Header()
        ball_pos.header.stamp = self.get_clock().now().to_msg()
        ball_pos.header.frame_id = 'map'
        self.publisher.publish(ball_pos)


def main(args=None):
    """ Spin the node. """
    rclpy.init()
    node = PositionCalculator()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()