#!/usr/bin/env python3

"""
ROS node for 2D odometry dump
"""

import sys
import rclpy
from nav_msgs.msg import Odometry
from tf_transformations import euler_from_quaternion

class DumpOdom:
    """ Node class """
    def __init__(self):

        # Node subscribers
        node.create_subscription(Odometry, 'odom', self.odom_cb)

    def odom_cb(self, msg):
        """ Odometry subscriber callback """
        quat = msg.pose.pose.orientation
        orientation_list = [quat.x, quat.y, quat.z, quat.w]
        (_, _, yaw) = euler_from_quaternion(orientation_list)

        print(str(rclpy.get_time()) + '\t' + \
        str(msg.pose.pose.position.x) + '\t' + \
        str(msg.pose.pose.position.y) + '\t' + str(yaw) + '\t' + \
        str(msg.twist.twist.linear.x) + '\t' + \
        str(msg.twist.twist.angular.z))

if __name__ == '__main__':
    rclpy.init(args=sys.argv)
    node = rclpy.create_node('dump_odom')

    DumpOdom()
    rclpy.spin()
