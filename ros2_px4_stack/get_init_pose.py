#!/usr/bin/env python3 

import os 
import rclpy 
from rclpy.node import Node 
from geometry_msgs.msg import PoseStamped 
import numpy as np

class InitialPose(Node):
    def __init__(self, namespace):
        super().__init__("InitialPose")

        # Create subscription 
        pose_sub = self.create_subscription(PoseStamped, namespace + "/world", self.pose_cb, 10)

    def pose_cb(self, msg):
        position, orientation = msg.pose.position, msg.pose.orientation
        x, y, z = position.x, position.y, position.z 
        roll, pitch, yaw = self.euler_from_quat(orientation)

        print(f'export INIT_X={x}')
        print(f'export INIT_Y={y}')
        print(f'export INIT_Z={z}')
        print(f'export INIT_ROLL={roll}')
        print(f'export INIT_PITCH={pitch}')
        print(f'export INIT_YAW={yaw}')

    def euler_from_quat(self, quaternion):
        """
        Converts quaternion (w in last place) to euler roll, pitch, yaw
        quaternion = [x, y, z, w]
        Bellow should be replaced when porting for ROS 2 Python tf_conversions is done.
        """
        x = quaternion.x
        y = quaternion.y
        z = quaternion.z
        w = quaternion.w

        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = np.arctan2(sinr_cosp, cosr_cosp)

        sinp = 2 * (w * y - z * x)
        sinp = np.clip(sinp, -1.0, 1.0)  # Clamping to handle numerical errors
        pitch = np.arcsin(sinp)

        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = np.arctan2(siny_cosp, cosy_cosp)

        return roll, pitch, yaw

def main(args=None):
    rclpy.init()
    node = InitialPose(namespace=os.environ["VEH_NAME"])
    rclpy.spin_once(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
    