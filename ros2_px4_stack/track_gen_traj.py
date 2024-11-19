#!/usr/bin/env python3 

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
import math
from threading import Thread
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion, Transform, Twist, Vector3
# from dynus_interfaces.msg import Goal
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

# from dynus_interfaces.msg import Goal
from trajectory_msgs.msg import MultiDOFJointTrajectory, MultiDOFJointTrajectoryPoint

from snapstack_msgs2.msg import Goal as GoalSnap
from snapstack_msgs2.msg import State as StateSnape

LOCAL_NAVIGATION = 0
GLOBAL_NAVIGATION = 1
NAVIGATION_MODES = [LOCAL_NAVIGATION, GLOBAL_NAVIGATION]

from .offboard_node import OffboardPathFollower

class SmoothTrajectoryTracker(OffboardPathFollower):

    def __init__(self, 
        node_name: str="smooth_trajectory_tracker", 
        navigation_mode: int=LOCAL_NAVIGATION,
    ):

        super().__init__(node_name=node_name, navigation_mode=navigation_mode)

        qos_profile = QoSProfile(depth=10)
        qos_profile.durability = DurabilityPolicy.VOLATILE
        qos_profile.reliability = ReliabilityPolicy.BEST_EFFORT

    def track_smooth_trajectory(self):
        self.track_trajectory()

def main(args=None):
    rclpy.init(args=args)
    traj_tracker = SmoothTrajectoryTracker()
    traj_tracker.track_smooth_trajectory()

if __name__ == '__main__':
    main()

