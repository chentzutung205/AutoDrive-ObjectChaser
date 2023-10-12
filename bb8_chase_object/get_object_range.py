import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point, Twist, Vector3
from sensor_msgs.msg import LaserScan
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy, QoSHistoryPolicy

import numpy as np
from math import pi, ceil


class RangeCatcher(Node):
    def __init__(self):
        super().__init__('get_object_range')

        image_qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
		    history=QoSHistoryPolicy.KEEP_LAST,
		    durability=QoSDurabilityPolicy.VOLATILE,
		    depth=1
		)

        self._bearing_angle_subscriber = self.create_subscription(Point, '/bearing_angle', self.object_callback, image_qos_profile)
        self._bearing_angle_subscriber

        self._lidar_subscriber = self.create_subscription(LaserScan, '/scan', self.lidar_callback, image_qos_profile)
        self._lidar_subscriber

        self.object_position_publisher = self.create_publisher(Twist, '/distance_and_angle', 10)

        self.theta = 0.0 # theta is the bearing angle of the object
        self.detect_object = False


    def object_callback(self, bearing):
        if bearing is not None:
            self.theta = bearing.x * (pi / 180)
            print(self.theta)


    def lidar_callback(self, posinfo):
        """
        LIDAR coordinate: counterclockwise = +theta
        """
        roi_min = (-30) * (pi / 180)
        roi_max = 30 * (pi / 180)
        if roi_min <= self.theta <= roi_max:
            roi = np.array(posinfo.ranges)
            roi = roi[~np.isnan(roi)]
            angle_min = posinfo.angle_min
            angle_inc = posinfo.angle_increment
            n = len(roi)

            if self.theta > 0:
                object_index = ceil((self.theta - angle_min) / angle_inc)
            else:
                object_index = n - ceil((abs(self.theta) - angle_min) / angle_inc)
            
            object_dis = roi[object_index]
            print("distance: ", object_dis)

            pos = Twist()
            pos.linear = Vector3()
            pos.linear.x = float(object_dis)
            pos.linear.y = 0.0
            pos.linear.z = 0.0
            pos.angular = Vector3()
            pos.angular.x = 0.0
            pos.angular.y = 0.0
            pos.angular.z = self.theta

            self.object_position_publisher.publish(pos)


def main(args=None):
    rclpy.init(args=args)
    get_object_range = RangeCatcher()
    rclpy.spin(get_object_range)
    get_object_range.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
