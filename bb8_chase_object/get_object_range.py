import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point, Twist, Vector3
from sensor_msgs.msg import LaserScan
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy, QoSHistoryPolicy

import numpy as np
from math import pi, floor, ceil


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
        self.bool_detect = False


    def object_callback(self, bearing):
        # decide it is detect
        if bearing.z != -1.0:
            self.theta = bearing.x * (pi / 180)
            self.bool_detect = True
        else:
            self.bool_detect = False


    def lidar_callback(self, posinfo):
        """
        LIDAR coordinate: counterclockwise = +theta
        """
        roi_min = (-30) * (pi / 180)
        roi_max = 30 * (pi / 180)

        # if roi_min <= self.theta and roi_max >= self.theta:
        if self.bool_detect is True:
            range = np.array(posinfo.ranges)
            range = range[~np.isnan(range)]
            n = len(range)
            
            angle_min = posinfo.angle_min
            angle_inc = posinfo.angle_increment
            
            object_index = ceil(abs(self.theta) / angle_inc) 
            if self.theta < 0:
                object_index = n - ceil(abs(self.theta) / angle_inc)
            
            object_dis = range[object_index]
            print("distance: ", object_dis)

            pos = Twist()
            pos.linear.x = float(object_dis)
            pos.angular.z = self.theta

            self.object_position_publisher.publish(pos)
        else:
            pos = Twist()
            pos.linear.x = 0.0
            pos.linear.z = -1.0
            pos.angular.z = 0.0


def main(args=None):
    rclpy.init(args=args)
    get_object_range = RangeCatcher()
    rclpy.spin(get_object_range)
    get_object_range.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
