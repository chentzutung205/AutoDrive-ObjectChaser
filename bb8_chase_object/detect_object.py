import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from geometry_msgs.msg import Point
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy, QoSHistoryPolicy
from cv_bridge import CvBridge

import cv2
import numpy as np


class ImageSubscriber(Node):

    def __init__(self):
        # Creates the node.
        super().__init__('image_subscriber')

        # Set up QoS Profiles for passing images over Wi-Fi
        image_qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            durability=QoSDurabilityPolicy.VOLATILE,
            depth=1
        )

        # Declare that the image_subscriber node is subscribing to the /camera/image/compressed topic
        self._image_subscriber = self.create_subscription(
            CompressedImage,
            '/image_raw/compressed',
            self._image_callback,
            image_qos_profile)
        self._image_subscriber  # Prevent unused variable warning

        self._pixel_coor_publisher = self.create_publisher(Point, '/pixel_coordinate', 10)
        self._bearing_angle_publisher = self.create_publisher(Point, '/bearing_angle', 10)


    def publish_pixel_coor(self, circles):
        if circles is not None:
            circles = np.uint16(np.around(circles))
            for circle in circles[0, :]:
                point = Point()
                point.x = float(circle[0])
                point.y = float(circle[1])
                point.z = 0.0
                self._pixel_coor_publisher.publish(point)

                # Coordinate the camera with the LIDAR
                bearing = Point()
                # Camera FOV angles in degrees
                horizontal_fov_degrees = 62.2
                vertical_fov_degrees = 48.8

                # Image resolution in pixels
                image_width = 320
                image_height = 240
                
                """
                Convert to bearing angle in degrees
                # angle range in frame: -30 degrees to 30 degrees
                # incorporate the coordinate with LIDAR
                # left to the middle: positive
                # right to the middle: negative
                """
                image_width_mid = image_width / 2
                bearing.x = (image_width_mid - point.x) / image_width * horizontal_fov_degrees
                bearing.y = 0.0
                bearing.z = 0.0

                self._bearing_angle_publisher.publish(bearing)


    def _image_callback(self, CompressedImage):
        # The "CompressedImage" is transformed to a color image in BGR space and is store in "_imgBGR"
        self._imgBGR = CvBridge().compressed_imgmsg_to_cv2(CompressedImage, "bgr8")
        # Image Processing to find circle
        imgGRAY = cv2.cvtColor(self._imgBGR, cv2.COLOR_BGR2GRAY)
        imgBLUR = cv2.medianBlur(imgGRAY, 5)
        circles = cv2.HoughCircles(imgBLUR, cv2.HOUGH_GRADIENT, 1, 80, param1=110, param2=63, minRadius=0, maxRadius=0)

        if circles is not None:
            circles = np.uint16(np.around(circles))
            for circle in circles[0, :]:
                # Print the pixel location
                xc, yc, r = circle[0], circle[1], circle[2]
                # draw the outer circle (bounding boxes)
                cv2.circle(self._imgBGR, (xc, yc), r, (255, 0, 0), 7)
                # draw the center of the circle
                cv2.circle(self._imgBGR, (xc, yc), 2, (0, 0, 255), 3)

        self.publish_pixel_coor(circles)



def main():
    rclpy.init()  # init routine needed for ROS2.
    image_subscriber = ImageSubscriber()  # Create class object to be used.
    rclpy.spin(image_subscriber)  # Trigger callback processing.

    # Clean up and shutdown.
    image_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
