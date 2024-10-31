#!/usr/bin/env python3

import rclpy 
from rclpy.node import Node 

from geometry_msgs.msg import PoseStamped

import tf_transformations
import math

class FixLivoxPose(Node):

    def __init__(self):
        super().__init__('fix_livox_pose')

        qos_profile = QoSProfile(depth=10)
        qos_profile.durability = DurabilityPolicy.VOLATILE
        qos_profile.reliability = ReliabilityPolicy.BEST_EFFORT


        self.subscription = self.create_subscription(PoseStamped, 
        "/dlio/odom_node/pose", self.pose_callback, qos_profile)

        self.publisher = self.create_publisher(PoseStamped, 
        "/livox/pose", qos_profile)
    
    def pose_callback(self, pose_msg):
        rotated_pose = self.rotate_pose(pose_msg, 135.0)
        self.publisher.publish(rotated_pose)

    def rotate_pose(self, pose, angle_deg):
        # Convert degrees to radians
        angle_rad = math.radians(angle_deg)

        # Create a quaternion for the rotation around the z-axis
        quaternion = tf_transformations.quaternion_from_euler(0, 0, -angler_rad)

        # Create a new PoseStamped for the rotated pose
        rotated_pose = pose
        rotated_pose.pose.orientation.x = quaternion[0]
        rotated_pose.pose.orientation.y = quaternion[1]
        rotated_pose.pose.orientation.z = quaternion[2]
        rotated_pose.pose.orientation.w = quaternion[3]

        # Update position using rotation formulas
        x = pose.pose.position.x 
        y = pose.pose.position.y
        rotated_pose.pose.position.x = x * math.cos(angle_rad) - y * math.sin(angle_rad)
        rotated_pose.pose.position.y = x * math.sin(angle_rad) + y * math.cos(angle_rad)

        return rotated_pose

def main(args=None):
    rclpy.init(args=args)
    fix_livox_pose = FixLivoxPose()
    rclpy.spin(fix_livox_pose)

if __name__ == '__main__':
    main()