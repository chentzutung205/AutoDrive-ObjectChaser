import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point, Twist, Vector3
from sensor_msgs.msg import LaserScan
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy, QoSHistoryPolicy

import numpy as np


ideal_dis = 0.4
dt = 0.15


class ChaseObject(Node):
    def __init__(self):
        super().__init__('chase_object')

        image_qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
		    history=QoSHistoryPolicy.KEEP_LAST,
		    durability=QoSDurabilityPolicy.VOLATILE,
		    depth=1
		)

        self._object_position_subscriber = self.create_subscription(Twist, '/distance_and_angle', self.object_callback, image_qos_profile)
        self._object_position_subscriber

        self.velocity_publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        self.e_old_l = 0.0
        self.e_integral_l = 0.0
        self.e_old_a = 0.0
        self.e_integral_a = 0.0


    def linear_pid(self, distance):
        # kp, ki, kd
        kp = 1.0
        ki = 0.0
        kd = 0.5

        e_l = distance - ideal_dis
        self.e_integral_l += e_l * dt
        e_dot_l = (e_l - self.e_old_l) / dt
        u_dis = kp * e_l + ki * self.e_integral_l + kd * e_dot_l

        self.e_old_l = e_l
        return u_dis

    
    def angular_pid(self, theta):
        # kp, ki, kd
        kp = 1.0
        ki = 0.0
        kd = 0.5

        e_a = theta - 0
        self.e_integral_a += e_a * dt
        e_dot_a = (e_a - self.e_old_a) / dt
        u_theta = kp * e_a + ki * self.e_integral_a + kd * e_dot_a

        self.e_old_a = e_a
        return u_theta


    def object_callback(self, posinfo):
        robot_velocity = Twist()

        # if the object is detected
        distance = posinfo.linear.x
        theta = posinfo.angular.z
        u_dis = self.linear_pid(distance)
        u_theta = self.angular_pid(theta)
        
        robot_velocity.linear.x = u_dis
        robot_velocity.angular.z = u_theta

        self.velocity_publisher.publish(robot_velocity)


def main(args=None):
    rclpy.init(args=args)
    chase_object = ChaseObject()
    rclpy.spin(chase_object)
    chase_object.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
