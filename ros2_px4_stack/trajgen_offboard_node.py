#! /usr/bin/env python3

"""
I'm commenting out the rclpy.sleep() functions because they don't have a 
ros2 equivalent (what rate are they sleeping on?). If this causes any trouble
we can try uncommenting the hacky wait_for_seconds() method and using that instead.
Be careful because the function is blocking and ros might yell at you 
"""

import rclpy
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from rclpy.node import Node
import math
from threading import Thread
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion, Transform, Twist, Vector3
from dynus_interfaces.msg import Goal
from dynus_interfaces.msg import State as StateDynus
from trajectory_msgs.msg import MultiDOFJointTrajectory, MultiDOFJointTrajectoryPoint

from snapstack_msgs2.msg import Goal as GoalSnap 
from snapstack_msgs2.msg import State as StateSnap

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
    # SetModeRequest,
    WaypointClear,
    WaypointPush,
)
from mavros_msgs.srv import CommandBool, SetMode #,SetModeRequest, CommandBoolRequest
from sensor_msgs.msg import NavSatFix, Imu
from pymavlink import mavutil

from typing import List, Tuple

from six.moves import xrange

from .base_mavros_interface import BasicMavrosInterface

LOCAL_NAVIGATION = 0  # x/y/z relative to home position
GLOBAL_NAVIGATION = 1  # lat/lon/alt
NAVIGATION_MODES = [LOCAL_NAVIGATION, GLOBAL_NAVIGATION]


class OffboardTrajgenFollower(BasicMavrosInterface):
    def __init__(
        self,
        node_name: str = "offboard_trajgen_follower",
        navigation_mode: int = LOCAL_NAVIGATION,
    ):
        super().__init__(node_name=node_name)
        self.navigation_mode = navigation_mode
        assert (
            self.navigation_mode in NAVIGATION_MODES
        ), f"Invalid navigation mode: {self.navigation_mode}"

        qos_profile = QoSProfile(depth=10)
        qos_profile.durability = DurabilityPolicy.VOLATILE
        qos_profile.reliability = ReliabilityPolicy.BEST_EFFORT

        # Traj gen sub/pub
        self.trajgen_goal_topic =  '/SQ01/goal'
        self.trajgen_state_topic = '/SQ01/state'
        self.trajgen_goal_sub = self.create_subscription(GoalSnap, self.trajgen_goal_topic, self.repub_traj_cb, qos_profile)
        self.trajgen_state_pub = self.create_publisher(StateSnap, self.trajgen_state_topic, 1)

        self.trajectory_setpoint = None
        self.received_trajectory_setpoint = None

        # Start thread for trajectory publisher 
        self.trajectory_publish_thread = Thread(
            target=self._publish_trajectory_setpoint, args=()
        )
        self.trajectory_publish_thread.daemon = True
        self.trajectory_publish_thread.start() 


    def wait_for_seconds(self, seconds):
        start_time = self.get_clock().now()
        while (self.get_clock().now() - start_time).nanoseconds < seconds * 1e9:
            rclpy.spin_once(self)

    def repub_traj_cb(self, msg):
        self.received_trajectory_setpoint = msg

        trajgen_state = StateSnap(
            header=Header(
                stamp=self.get_clock().now().to_msg(),
                frame_id="map"
            ),
            pos=Point(
                x=self.local_position.pose.position.x,
                y=self.local_position.pose.position.y,
                z=self.local_position.pose.position.z
            ),
            quat=Quaternion(
                x=self.local_position.pose.orientation.x,
                y=self.local_position.pose.orientation.y,
                z=self.local_position.pose.orientation.z,
                w=self.local_position.pose.orientation.w
            )
        )

        self.trajgen_state_pub.publish(trajgen_state)

    def _publish_trajectory_setpoint(self):
        rate = 100 #Hz
        rate = self.create_rate(rate)
        while rclpy.ok():
            if (
                self.navigation_mode == LOCAL_NAVIGATION
                and self.trajectory_setpoint is not None
            ):
                self.setpoint_traj_pub.publish(self.trajectory_setpoint)
            rate.sleep()

    def _pack_into_traj_gen(self, point: GoalSnap):
        assert self.navigation_mode == LOCAL_NAVIGATION, (
            f"Invalid navigation mode: {self.navigation_mode}."
            f"Only local navigation is supported for this method"
        )

        trajectory_points = [MultiDOFJointTrajectoryPoint(
            transforms=[Transform(
                translation=Vector3(
                    x=point.p.x,
                    y=point.p.y,
                    z=point.p.z,
                ),
                rotation=Quaternion(
                    x=yaw_to_quaternion(point.psi)[0],
                    y=yaw_to_quaternion(point.psi)[1],
                    z=yaw_to_quaternion(point.psi)[2],
                    w=yaw_to_quaternion(point.psi)[3]
                )
            )],
            velocities=[Twist(
                linear=Vector3(
                    x=point.v.x,
                    y=point.v.y,
                    z=point.v.z
                ),
                angular=Vector3(
                    x=0.0,
                    y=0.0,
                    z=point.dpsi
                )
            )],
            accelerations=[Twist(
                linear=Vector3(
                    x=point.a.x,
                    y=point.a.y,
                    z=point.a.z
                )
            )]

        )]

        trajectory_msg = MultiDOFJointTrajectory(
            header=Header(
                stamp=self.get_clock().now().to_msg(),
                frame_id="map"
            ),
            points=trajectory_points
        )

        return trajectory_msg


    def track_trajectory(self):
        # wait 1 second for FCU connection
        self.wait_for_seconds(1)
        while rclpy.ok():
            if self.received_trajectory_setpoint != None:
                self.trajectory_setpoint = self._pack_into_traj_gen(self.received_trajectory_setpoint)
                self.wait_for_seconds(0.2)

    
########################
### Helper Functions ###
########################

def yaw_to_quaternion(yaw):
    qx = 0.0
    qy = 0.0
    qz = math.sin(yaw / 2.0)
    qw = math.cos(yaw / 2.0)

    return [qx, qy, qz, qw]



def main():
    rclpy.init()
    node_name = "offboard_trajgen_follower"
    node = OffboardTrajgenFollower(
        node_name=node_name, navigation_mode=LOCAL_NAVIGATION
    )
    rclpy.spin(node)
    
if __name__ == "__main__":
    main()

