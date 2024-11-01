#!/usr/bin/env python3

import rclpy 
from rclpy.node import Node 
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from geometry_msgs.msg import PoseStamped
import math

from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from tf2_ros import Buffer, TransformListener 
from tf2_geometry_msgs import do_transform_pose 

import numpy as np 

class FixLivoxPose(Node):

    def __init__(self):
        super().__init__('fix_livox_pose')

        qos_profile = QoSProfile(depth=10)
        qos_profile.durability = DurabilityPolicy.VOLATILE
        qos_profile.reliability = ReliabilityPolicy.BEST_EFFORT


        self.spoof_pose = PoseStamped()
        self.spoof_pose.header.frame_id = "base_link"
        self.spoof_pose.pose.position.x = 0.0
        self.spoof_pose.pose.position.y = 0.0
        self.spoof_pose.pose.position.z = 0.0

        # Make pose subscription
        self.subscription = self.create_subscription(PoseStamped, 
        "/dlio/odom_node/pose", self.pose_callback, qos_profile)

        # Make pose publisher 
        self.publisher = self.create_publisher(PoseStamped, 
        "/livox/pose", qos_profile)

        # Make transform listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
    
    def pose_callback(self, pose_msg):
        new_pose = self.apply_transform(pose_msg)
        self.publisher.publish(new_pose)

    def apply_transform(self, in_pose):    
        # Get static transform between PX01 and base_link
        pose_stamped = PoseStamped()
        pose_stamped.pose = in_pose
        if not self.tf_buffer.can_transform("PX01", "base_link", self.get_clock().now()):
            self.get_logger().info(f"Waiting for transform")
            rclpy.spin_once(self, timeout_sec=1.0)

        self.get_logger().info(f"in_pose: {pose_stamped}")
        transform = self.tf_buffer.lookup_transform('PX01', 'base_link', self.get_clock().now())

        # Get PX01 pose from base_link pose (in_pose) and their relative transform
        transformed_pose = do_transform_pose(pose_stamped, transform)

        return transformed_pose

    def quaternion_from_euler(self, ai, aj, ak):
        ai /= 2.0
        aj /= 2.0
        ak /= 2.0
        ci = math.cos(ai)
        si = math.sin(ai)
        cj = math.cos(aj)
        sj = math.sin(aj)
        ck = math.cos(ak)
        sk = math.sin(ak)
        cc = ci*ck
        cs = ci*sk
        sc = si*ck
        ss = si*sk

        q = np.empty((4, ))
        q[0] = cj*sc - sj*cs
        q[1] = cj*ss + sj*cc
        q[2] = cj*cs - sj*sc
        q[3] = cj*cc + sj*ss

        return q

def main(args=None):
    rclpy.init(args=args)
    fix_livox_pose = FixLivoxPose()
    rclpy.spin(fix_livox_pose)

if __name__ == '__main__':
    main()