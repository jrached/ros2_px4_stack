#! /usr/bin/env python3
"""A script to republish motion capture data from a ROS topic to a different topic."""

"""
Juan Notes:
    1. Might want to remove "/optitrack" from mocap_sub_topic_name (just /PX01/world) 
    2. Might have some issues with threading and global executors.
       Go back to humble ros2 python migrating guide to look at their code.
"""

import rclpy
from rclpy.node import Node
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, TwistStamped, PoseWithCovarianceStamped, TwistWithCovarianceStamped
import threading
from threading import Thread
from nav_msgs.msg import Odometry
# from threading import Thread
# from rclpy.executors import ExternalShutdownException 


class LivoxRepublisher(Node):
    def __init__(self, pub_hz=50.0):
        super().__init__('livox_publisher')

        namespace = self.get_namespace()

        self.livox_odom_sub_topic_name = self.declare_parameter(
            "~livox_odom_sub_topic_name", namespace + "/dlio/odom_node/odom"
        ).value 

        self.livox_pose_pub_topic_name = self.declare_parameter(
            "~livox_pose_pub_topic_name",  namespace + "/mavros/vision_pose/pose_cov"
        ).value

        self.livox_twist_pub_topic_name = self.declare_parameter(
            "~livox_twist_pub_topic_name", namespace + "/mavros/vision_speed/speed_twist_cov"
        ).value 

        # Create subscription to lidar odometry 
        self._livox_odom_sub = self.create_subscription(Odometry, 
            self.livox_odom_sub_topic_name, self._livox_odom_cb, 10)
        self._last_pose_msg = None
        self._last_twist_msg = None 

        # Publish pose and twist with covariance at the specified rate in a separate thread
        self._pub_hz = pub_hz
        self._livox_pose_pub = self.create_publisher(PoseWithCovarianceStamped, self.livox_pose_pub_topic_name, 10)
        self._livox_twist_pub = self.create_publisher(TwistWithCovarianceStamped, self.livox_twist_pub_topic_name, 10)
        self._pub_thread = Thread(target=self._publish_loop, args=())
        self._pub_thread.daemon = True
        self._pub_thread.start()

    def _livox_odom_cb(self, msg):
        # Populate pose_cov msg
        pose_msg = PoseWithCovarianceStamped()
        pose_msg.header.stamp = msg.header.stamp
        pose_msg.header.frame_id = msg.header.frame_id 
        pose_msg.pose = msg.pose 
        self._last_pose_msg = pose_msg

        # Populate twist_cov msg
        twist_msg = TwistWithCovarianceStamped()
        twist_msg.header.stamp = msg.header.stamp 
        twist_msg.header.frame_id = msg.header.frame_id 
        twist_msg.twist = msg.twist 
        self._last_twist_msg = twist_msg  

    def _publish_loop(self):
        rate = self.create_rate(self._pub_hz)
        while rclpy.ok():
            if self._last_pose_msg:
                self._livox_pose_pub.publish(self._last_pose_msg)
            if self._last_twist_msg: 
                self._livox_twist_pub.publish(self._last_twist_msg)
            rate.sleep()

def main():
    rclpy.init()
    node = LivoxRepublisher(pub_hz=50.0)
    rclpy.spin(node)

if __name__ == "__main__":
    main()
    
    

