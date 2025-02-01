#! /usr/bin/env python3
import os
import rclpy
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from rclpy.node import Node
import math
from threading import Thread
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion, Transform, Twist, Vector3
from dynus_interfaces.msg import Goal
from dynus_interfaces.msg import State as StateDynus
from trajectory_msgs.msg import MultiDOFJointTrajectory, MultiDOFJointTrajectoryPoint
from scipy.spatial.transform import Rotation
import numpy as np

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


class OffboardDynusFollower(BasicMavrosInterface):
    def __init__(
        self,
        node_name: str = "offboard_dynus_follower",
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

        self.trajectory_setpoint = None
        self.received_trajectory_setpoint = None
        self.flight_state = "TAKEOFF"
        self.alt_ = 1.0

        # Vehicle dynamics
        self.m = 2.744
        self.g = 9.81

        # Dynus subscriptions/publishers 
        veh = os.environ.get("VEH_NAME")
        self.dynus_goal_topic = f'/{veh}/goal'
        self.dynus_traj_sub = self.create_subscription(Goal, self.dynus_goal_topic, self.dynus_cb, qos_profile)
        
        # Start thread for trajectory publisher 
        self.trajectory_publish_thread = Thread(
            target=self.takeoff_cb)
        self.trajectory_publish_thread.daemon = True
        self.trajectory_publish_thread.start() 


    def dynus_cb(self, msg):
        self.received_trajectory_setpoint = msg

        if self.flight_state == "TRAJECTORY" and self.navigation_mode == LOCAL_NAVIGATION: 
            self.trajectory_setpoint = self._pack_into_traj(self.received_trajectory_setpoint)
            self.setpoint_traj_pub.publish(self.trajectory_setpoint)

    def takeoff_cb(self):
        freq = 50 #Hz
        rate = self.create_rate(freq)

        takeoff_pos = self.point_to_traj([self.local_position.pose.position.x, self.local_position.pose.position.y, self.alt_])

        while rclpy.ok():
            if self.flight_state == "TAKEOFF":
                self.get_logger().info("Taking Off")

                if self.navigation_mode == LOCAL_NAVIGATION:
                    self.setpoint_traj_pub.publish(takeoff_pos)

                if (self.traj_point_reached(takeoff_pos)
                    and self.received_trajectory_setpoint is not None):
                    self.get_logger().info("Takeoff Complete. Following Trajectory.")
                    self.flight_state = "TRAJECTORY"

            elif self.flight_state == "TRAJECTORY":
                break
            
            rate.sleep()

    def point_to_traj(self, point: List):
        """
        Converts a single point into a mavros trajectory object.
        """
        traj_point = MultiDOFJointTrajectoryPoint()
            
        transform = Transform()
        transform.translation.x = point[0]
        transform.translation.y = point[1]
        transform.translation.z = point[2]
        traj_point.transforms = [transform]
        
        twist = Twist()
        traj_point.velocities = [twist]
        traj = MultiDOFJointTrajectory()
        traj.points = [traj_point]

        return traj

    def _pack_into_traj(self, point: Goal):
        """
        Converts a dynus trajectory into a mavros trajectory point by point. 
        """
        assert self.navigation_mode == LOCAL_NAVIGATION, (
            f"Invalid navigation mode: {self.navigation_mode}."
            f"Only local navigation is supported for this method"
        )

        quat = self.get_orientation(point)
        p, q, r = self.get_angular(point)

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

    def get_drone_frame(self, point):
        # Construct differentially flat vectors 
        sigma = np.array([[point.p.x, point.p.y, point.p.z, point.yaw]]).T
        sigma_dot_dot = np.array([[point.a.x, point.a.y, point.a.z, 0]]).T

        # Compute z_B 
        t = np.array([[sigma_dot_dot[0][0], sigma_dot_dot[1][0], sigma_dot_dot[2][0] + self.g]]).T
        z_B = t / np.linalg.norm(t) 

        # Compute intermediate yaw vector x_C
        x_C = np.array([[np.cos(sigma[3][0]), np.sin(sigma[3][0]), 0]]).T

        # Compute x-axis and y-axis of drone frame measured in world frame
        cross_prod = np.cross(z_B.T[0], x_C.T[0]) 
        y_B = np.array([cross_prod / np.linalg.norm(cross_prod)]).T 
        x_B = np.array([np.cross(y_B.T[0], z_B.T[0])]).T 

        return x_B, y_B, z_B 

    def get_orientation(self, point):
        x_B, y_B, z_B = self.get_drone_frame(point)

        # Populate rotation matrix of drone frame measured from world frame
        R_W_B = np.hstack((x_B, y_B, z_B))

        return Rotation.from_matrix(R_W_B).as_quat() # As [x, y, z, w] vector

    def get_angular(self, point): 
        z_W = np.array([[0, 0, 1]]).T 
        x_B, y_B, z_B = self.get_drone_frame(point)
        jerk = np.array([[point.j.x, point.j.y, point.j.z]]).T 
        dpsi = point.dyaw 

        # Compute u1 
        f_des = self.m * self.g * z_W + self.m * np.array([[point.a.x, point.a.y, point.a.z]]).T
        u1 = (f_des.T @ z_B)[0, 0]

        # Compute h_om
        h_om = self.m / u1 * (jerk - (z_B.T @ jerk)[0, 0] * z_B)

        # Compute angular velocities 
        p = float(-(h_om.T @ y_B)[0, 0])
        q = float((h_om.T @ x_B)[0, 0])
        r = float(dpsi * (z_W.T @ z_B)[0, 0]) 

        return p, q, r


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
    node_name = "offboard_dynus_follower"
    node = OffboardDynusFollower(
        node_name=node_name, navigation_mode=LOCAL_NAVIGATION
    )
    rclpy.spin(node)
    
if __name__ == "__main__":
    main()

