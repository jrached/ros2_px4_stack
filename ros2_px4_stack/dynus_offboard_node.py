#! /usr/bin/env python3

"""
I'm commenting out the rclpy.sleep() functions because they don't have a 
ros2 equivalent (what rate are they sleeping on?). If this causes any trouble
we can try uncommenting the hacky wait_for_seconds() method and using that instead.
Be careful because the function is blocking and ros might yell at you 
"""
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

        # Dynus subscriptions/publishers 
        veh = os.environ.get("VEH_NAME")
        self.dynus_goal_topic = f'/{veh}/goal'
        self.dynus_traj_sub = self.create_subscription(Goal, self.dynus_goal_topic, self.dynus_cb, qos_profile)
        
        # Start thread for trajectory publisher 
        self.trajectory_publish_thread = Thread(
            target=self._publish_trajectory_setpoint, args=()
        )
        self.trajectory_publish_thread.daemon = True
        self.trajectory_publish_thread.start() 


    # Method to wait for FCU connection 
    def wait_for_seconds(self, seconds):
        start_time = self.get_clock().now()
        while (self.get_clock().now() - start_time).nanoseconds < seconds * 1e9:
            rclpy.spin_once(self)

    def dynus_cb(self, msg):
        self.received_trajectory_setpoint = msg

    def _publish_trajectory_setpoint(self):
        rate = 50 #Hz
        rate = self.create_rate(rate)
        while rclpy.ok():
            if (
                self.navigation_mode == LOCAL_NAVIGATION
                and self.trajectory_setpoint is not None
            ):
                self.setpoint_traj_pub.publish(self.trajectory_setpoint)
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

        quat = get_orientation(point)
        self.get_logger().info(f"Computed Quaternion: {quat}")

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
                    x=0.0,
                    y=0.0,
                    z=point.dyaw
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

    def takeoff_and_track_trajectory(self, altitude):

        # wait 1 second for FCU connection
        self.wait_for_seconds(1)

        flight_state = "TAKEOFF"
        takeoff_pos = self.point_to_traj([self.local_position.pose.position.x, self.local_position.pose.position.y, altitude])

        while rclpy.ok():
            if flight_state == "TAKEOFF":
                self.get_logger().info("Taking Off")

                self.trajectory_setpoint = takeoff_pos

                if (self.traj_point_reached(takeoff_pos)
                    and self.received_trajectory_setpoint is not None):
                    self.get_logger().info("Takeoff Complete")
                    flight_state = "TRAJECTORY"
                    init_pos = takeoff_pos

            elif flight_state == "TRAJECTORY":
                self.get_logger().info("Following Trajectory")
                if self.received_trajectory_setpoint:
                    self.trajectory_setpoint = self._pack_into_traj(self.received_trajectory_setpoint)

                # If trajectory is over 
                if (self.count_publishers(self.dynus_goal_topic) == 0):
                    self.get_logger().info("Returning to Initial Position")
                    flight_state = "RETURN"
            
            elif flight_state == "RETURN":
                self.trajectory_setpoint = init_pos

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

def get_drone_frame(point):
    g = 9.81

    # Construct differentially flat vectors 
    sigma = np.array([[point.p.x, point.p.y, point.p.z, point.yaw]]).T
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
    m = 2.6 # TODO get actual value 
    g = 9.81
    z_W = np.array([[0, 0, 1]]).T 
    x_B, y_B, z_B = get_drone_frame(point)
    jerk = np.array([[point.j.x, point.j.y, point.j.z]]).T 
    dpsi = point.dyaw 

    # Compute u1 
    f_des = m * g * z_W + m * np.array([[point.a.x, point.a.y, point.a.z]]).T
    u1 = (f_des.T @ z_B)[0, 0]

    # Compute h_om
    h_om = m / u1 * (jerk - (z_B.T @ jerk)[0, 0] * z_B)

    # Compute angular velocities 
    p = - (h_om.T @ y_B)[0, 0]
    q = (h_om.T @ x_B)[0, 0]
    r = dpsi * (z_W.T @ z_B)[0, 0] 

    return p, q, r


def main():
    rclpy.init()
    node_name = "offboard_dynus_follower"
    node = OffboardDynusFollower(
        node_name=node_name, navigation_mode=LOCAL_NAVIGATION
    )
    rclpy.spin(node)
    
if __name__ == "__main__":
    main()

