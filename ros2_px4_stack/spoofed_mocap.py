#!/usr/bin/env python

import rclpy
from rclpy.node import Node
import tf2_ros
from geometry_msgs.msg import PoseStamped

agent_name = 'PX01'

if __name__ == '__main__':
    rclpy.init()
    node = rclpy.create_node('spoofed_mocap')

    rate = node.create_rate(30)
    # pub = rclpy.Publisher(f'/{agent_name}/world', PoseStamped, queue_size=10)
    pub = node.create_publisher(PoseStamped, f'/{agent_name}/world', 10)

    msg = PoseStamped()
    msg.pose.position.x = 0
    msg.pose.position.y = 0
    msg.pose.position.z = 0
    msg.pose.orientation.x = 0
    msg.pose.orientation.y = 0
    msg.pose.orientation.z = 0
    msg.pose.orientation.w = 1

    while rclpy.ok():
        msg.header.stamp = node.get_clock().now()
        pub.publish(msg)
        rate.sleep()

    rclpy.spin(node)
