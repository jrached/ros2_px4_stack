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
import numpy as np 
from scipy.signal import chirp 

from trajectory_msgs.msg import MultiDOFJointTrajectory, MultiDOFJointTrajectoryPoint

from snapstack_msgs2.msg import Goal as GoalSnap
from snapstack_msgs2.msg import State as StateSnape

LOCAL_NAVIGATION = 0
GLOBAL_NAVIGATION = 1
NAVIGATION_MODES = [LOCAL_NAVIGATION, GLOBAL_NAVIGATION]

from .trajgen_offboard_node import OffboardTrajgenFollower
from .sine_sweep_offboard_node import SineSweep  

class SmoothTrajectoryTracker(OffboardTrajgenFollower):

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

class SineSweepTracker(SineSweep):

    def __init__(self, 
        node_name: str="sine_sweep_tracker", 
        navigation_mode: int=LOCAL_NAVIGATION,
    ):

        super().__init__(node_name=node_name, navigation_mode=navigation_mode)

        qos_profile = QoSProfile(depth=10)
        qos_profile.durability = DurabilityPolicy.VOLATILE
        qos_profile.reliability = ReliabilityPolicy.BEST_EFFORT

    def generate_sine_sweep(self, wave_duration=6.0, sampling_rate=100):
        # choose frequency range. 
        omegas = 2 * np.pi * np.arange(0.1, 5.0, 0.1)
        
        sine_sweep = []
        # For each frequencey generate sine wave
        # for omega in omegas: 
        #     wave_duration = 4 * np.pi / omega # Run each frequency for 2 cycles 
        #     duration = np.linspace(0, wave_duration, int(sampling_rate * wave_duration)) 
        #     # For each t in wave_duration append sin(omega * t) to a list 
        #     for t in duration: 
        #         sine_sweep.append(np.sin(omega * t))

        total_duration = 60
        t = np.linspace(0, total_duration, int(sampling_rate * total_duration))
        sine_sweep = chirp(t, f0=0.01, f1=3.0, t1=total_duration, method='log')

        return list(sine_sweep)  

    def track_trajectory(self, altitude=1.5):
        sine_sweep = self.generate_sine_sweep()
        self.track_sine_sweep(sine_sweep, altitude)


def main(args=None):
    rclpy.init(args=args)

    sine_sweep = True 
    if sine_sweep:
        traj_tracker = SineSweepTracker()
        traj_tracker.track_trajectory()
    else:
        traj_tracker = SmoothTrajectoryTracker()
        traj_tracker.track_smooth_trajectory()

if __name__ == '__main__':
    main()

