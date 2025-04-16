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
from trajectory_msgs.msg import MultiDOFJointTrajectory, MultiDOFJointTrajectoryPoint
import numpy as np 
from scipy.spatial.transform import Rotation 

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

        quat = get_orientation(point)
        p, q, r = get_angular(point) 

        trajectory_points = [MultiDOFJointTrajectoryPoint(
            transforms=[Transform(
                translation=Vector3(
                    x=point.p.x,
                    y=point.p.y,
                    z=point.p.z,
                ),
                rotation=Quaternion(
                    x=quat[0],
                    y=quat[1],
                    z=quat[2],
                    w=quat[3]
                )
            )],
            velocities=[Twist(
                linear=Vector3(
                    x=point.v.x,
                    y=point.v.y,
                    z=point.v.z
                ),
                angular=Vector3(
                    x=p,
                    y=q,
                    z=r
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
def get_drone_frame(point):
    g = 9.81

    # Construct differentially flat vectors 
    sigma = np.array([[point.p.x, point.p.y, point.p.z, point.psi]]).T
    sigma_dot_dot = np.array([[point.a.x, point.a.y, point.a.z, 0]]).T

    # Compute z_B 
    t = np.array([[sigma_dot_dot[0][0], sigma_dot_dot[1][0], sigma_dot_dot[2][0] + g]]).T
    z_B = t / np.linalg.norm(t) 

    # Compute intermediate yaw vector x_C
    x_C = np.array([[np.cos(sigma[3][0]), np.sin(sigma[3][0]), 0]]).T

    # Compute x-axis and y-axis of drone frame measured in world frame
    cross_prod = np.cross(z_B.T[0], x_C.T[0]) 
    y_B = np.array([cross_prod / np.linalg.norm(cross_prod)]).T 
    x_B = np.array([np.cross(y_B.T[0], z_B.T[0])]).T 

    return x_B, y_B, z_B 

def get_orientation(point):
    x_B, y_B, z_B = get_drone_frame(point)

    # Populate rotation matrix of drone frame measured from world frame
    R_W_B = np.hstack((x_B, y_B, z_B))

    return Rotation.from_matrix(R_W_B).as_quat() # As [x, y, z, w] vector

def get_angular(point):
    m = 2.906 
    g = 9.81
    z_W = np.array([[0, 0, 1]]).T 
    x_B, y_B, z_B = get_drone_frame(point)
    jerk = np.array([[point.j.x, point.j.y, point.j.z]]).T 
    dpsi = point.dpsi

    # Compute u1 
    f_des = m * g * z_W + m * np.array([[point.a.x, point.a.y, point.a.z]]).T
    u1 = (f_des.T @ z_B)[0, 0]

    # Compute h_om
    h_om = m / u1 * (jerk - (z_B.T @ jerk)[0, 0] * z_B)

    # Compute angular velocities 
    p = float(- (h_om.T @ y_B)[0, 0])
    q = float((h_om.T @ x_B)[0, 0])
    r = float(dpsi * (z_W.T @ z_B)[0, 0]) 

    return p, q, r

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

