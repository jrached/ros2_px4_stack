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
from geometry_msgs.msg import PoseStamped, TwistStamped
import threading
from threading import Thread
# from threading import Thread
# from rclpy.executors import ExternalShutdownException 


class LivoxRepublisher(Node):
    def __init__(self, pub_hz=30.0):
        super().__init__('livox_publisher')

        self.livox_sub_topic_name = self.declare_parameter(
            
            "~livox_sub_topic_name", "/dlio/odom_node/odom"
        ).value

        self.livox_pub_topic_name = self.declare_parameter(
            "~livox_pub_topic_name",  "/mavros/vision_pose/pose"
        ).value

        self._livox_sub = self.create_subscription(PoseStamped,
            self.livox_sub_topic_name, self._livox_cb, 10)
        self._last_msg = None

        # Publish at the specified rate in a separate thread
        self._pub_hz = pub_hz
        self._livox_pub = self.create_publisher(PoseStamped, self.livox_pub_topic_name, 10)
        self._pub_thread = Thread(target=self._publish_loop, args=())
        self._pub_thread.daemon = True
        self._pub_thread.start()


    def _livox_cb(self, msg):
        self._last_msg = msg

    def _publish_loop(self):
        rate = self.create_rate(self._pub_hz)
        while rclpy.ok():
            if self._last_msg:
                self._livox_pub.publish(self._last_msg)
            rate.sleep()

def main():
    rclpy.init()
    node = LivoxRepublisher(pub_hz=20.0)
    rclpy.spin(node)

if __name__ == "__main__":
    main()
    
    

