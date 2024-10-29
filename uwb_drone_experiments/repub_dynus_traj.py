#!/usr/bin/env python3 

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
import math
from threading import Thread
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion, Transform, Twist, Vector3
from std_msgs.msg import Header
from mavros import mavlink
from mavros_msgs.msg import (
    Altitude,
    ExtendedState,
    HomePosition,
    ParamValue,
    State,
    WaypointList,
    Waypoint,
    Mavlink,
    CommandCode,
)
from mavros_msgs.srv import (
    CommandBool,
    ParamGet,
    ParamSet,
    SetMode,
    WaypointClear,
    WaypointPush,
)
from mavros_msgs.srv import CommandBool, SetMode
from sensor_msgs.msg import NavSatFix, Imu
from pymavlink import mavutil
from typing import List, Tuple
from six.moves import xrange

from dynus_interfaces.msg import Goal
from trajectory_msgs.msg import MultiDOFJointTrajectory, MultiDOFJointTrajectoryPoint

LOCAL_NAVIGATION = 0
GLOBAL_NAVIGATION = 1
NAVIGATION_MODES = [LOCAL_NAVIGATION, GLOBAL_NAVIGATION]

from .offboard_node import OffboardPathFollower

class SmoothTrajectoryPublisher(OffboardPathFollower):

    def __init__(self, 
        node_name: str="smooth_trajectory_publisher", 
        navigation_mode: int=LOCAL_NAVIGATION
    ):

        super().__init__(node_name=node_name, navigation_mode=navigation_mode)

        qos_profile = QoSProfile(depth=10)
        qos_profile.durability = DurabilityPolicy.VOLATILE
        qos_profile.reliability = ReliabilityPolicy.BEST_EFFORT

        # Make callback that subscribes to rosbag at /NX01/goal
        self.traj_topic = '/NX01/goal'
        self.create_subscription(Goal, self.traj_topic, self.repub_traj_cb, qos_profile)

        # Make publisher to publish new traj
        self.new_traj = MultiDOFJointTrajectory()
        self.new_traj_pub_ = self.create_publisher(MultiDOFJointTrajectory, '/mavros/setpoint_trajectory/local', qos_profile)

        # Create and start new thread
        # self.thread = Thread(target=self.timer_cb)
        # self.thread.start()


    def repub_traj_cb(self, msg):
        self.new_traj = self._pack_into_traj_setpoints(msg)
        self.new_traj_pub_.publish(self.new_traj)
        # self.get_logger().info(f"Callback is getting: {self.new_traj}")

    # def timer_cb(self):
    #     rate = self.create_rate(90)
    #     self.new_traj_pub_.publish(self.new_traj)
    #     rate.sleep()
        # self.get_logger().info("Nothing to see here")


def main(args=None):
    rclpy.init(args=args)
    trajectory_publisher = SmoothTrajectoryPublisher()
    rclpy.spin(trajectory_publisher)
    trajectory_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

