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


class OffboardSetpointFollower(BasicMavrosInterface):
    def __init__(
        self,
        node_name: str = "offboard_setpoint_follower",
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

        self.current_setpoint = None
        
        # Start thread for setpoint publisher 
        self.setpoint_publish_thread = Thread(
            target=self._publish_current_setpoint, args=()
        ) 
        self.setpoint_publish_thread.daemon = True
        self.setpoint_publish_thread.start()

        # set up capacity to listen for custom setpoints
        self.outside_setpoint_sub = self.create_subscription(PoseStamped,
            "offboard/setpoint", self._outside_setpoint_callback, qos_profile
        )
        self.received_outside_setpoint = False

    def _outside_setpoint_callback(self, msg: PoseStamped):
        self.current_setpoint = msg
        self.received_outside_setpoint = True

    def track_setpoints(self, setpoints: List[PoseStamped], altitude: float):
        # This mode requires position or pose/attitude information - e.g. GPS, optical flow, visual-inertial odometry, mocap, etc.
        # RC control is disabled except to change modes (you can also fly without any manual controller at all by setting the parameter COM_RC_IN_MODE to 4: Stick input disabled).
        # The vehicle must be already be receiving a stream of MAVLink setpoint messages or ROS 2 OffboardControlMode messages before arming in offboard mode or switching to offboard mode when flying.
        # The vehicle will exit offboard mode if MAVLink setpoint messages or OffboardControlMode are not received at a rate of > 2Hz.
        # Not all coordinate frames and field values allowed by MAVLink are supported for all setpoint messages and vehicles. Read the sections below carefully to ensure only supported values are used.

        """NOTE
        -The vehicle must be already be receiving a stream of MAVLink setpoint messages or ROS 2 OffboardControlMode messages before arming in offboard mode or switching to offboard mode when flying.
        -The vehicle will exit offboard mode if MAVLink setpoint messages or OffboardControlMode are not received at a rate of > 2Hz.
        -Not all coordinate frames and field values allowed by MAVLink are supported for all setpoint messages and vehicles. Read the sections below carefully to ensure only supported values are used.
        """
        if self.received_outside_setpoint:
            self.get_logger().error("Received outside setpoint. Ignoring track_setpoints command")
            return

        # wait 1 second for FCU connection
        self.wait_for_seconds(1)

        flight_state, completed_laps, max_laps = -1, 0, 2
        while rclpy.ok():

            if flight_state == -1:
                x_init, y_init = self.local_position.pose.position.x, self.local_position.pose.position.y
                flight_state = 0

            elif flight_state == 0: 
                #Creates takeoff setpoint
                takeoff_vertices = [(x_init, y_init, altitude)]
                takeoff_setpoints = self._pack_into_setpoints(takeoff_vertices)
               
                #Sets current setpoint to takeoff (this gets published in publish_current_setpoint())
                self.current_setpoint = takeoff_setpoints[0]

                #If takeoff complete move onto the next state
                if self.setpoint_reached(takeoff_setpoints[0]):
                    flight_state = 1
                    cur_setpoint_idx = 0
                    self.current_setpoint = setpoints[cur_setpoint_idx]
        
            elif flight_state == 1:
                # if we've reached the current setpoint, move to the next one,
                # looping back to the first one if necessary 
                if self.setpoint_reached(setpoints[cur_setpoint_idx]):
                    cur_setpoint_idx = (cur_setpoint_idx + 1) % len(setpoints)
                    
                    # If lap completed increase lap counter
                    if setpoints[cur_setpoint_idx] == setpoints[0]:
                        completed_laps += 1 

                self.current_setpoint = setpoints[cur_setpoint_idx]

                # If all laps complete move back to takeoff side
                if completed_laps >= max_laps:
                    flight_state = 2

            elif flight_state == 2:
                self.current_setpoint = takeoff_setpoints[0]
            
            # Wait 0.2 seconds to publish at right frequency
            self.wait_for_seconds(0.2)

    # Method to wait for FCU connection 
    def wait_for_seconds(self, seconds):
        start_time = self.get_clock().now()
        while (self.get_clock().now() - start_time).nanoseconds < seconds * 1e9:
            rclpy.spin_once(self)

    def _publish_current_setpoint(self):
        """Publishes the current setpoint"""
        setpoint_publish_rate = 20  # Hz
        rate = self.create_rate(setpoint_publish_rate)
        while rclpy.ok():
            if (
                self.navigation_mode == LOCAL_NAVIGATION
                and self.current_setpoint is not None
            ):
                self.setpoint_position_pub.publish(self.current_setpoint)
            rate.sleep()

    def _pack_into_setpoints(
        self, points: List[Tuple[float, float, float]]
    ) -> List[PoseStamped]:
        assert self.navigation_mode == LOCAL_NAVIGATION, (
            f"Invalid navigation mode: {self.navigation_mode}. "
            f"Only local navigation is supported for this method"
        )

        setpoints = [
            PoseStamped(
                header=Header(
                    stamp=self.get_clock().now().to_msg(),
                    frame_id="map",
                ),
                pose=Pose(
                    position=Point(
                        x=point[0],
                        y=point[1],
                        z=point[2],
                    ),
                    orientation=Quaternion(
                        x=0.0,
                        y=0.0,
                        z=0.0,
                        w=1.0,
                    ),
                ),
            )
            for point in points
        ]

        return setpoints


def main():
    rclpy.init()
    node_name = "offboard_setpoint_follower"
    node = OffboardSetpointFollower(
        node_name=node_name, navigation_mode=LOCAL_NAVIGATION
    )
    rclpy.spin(node)
    
if __name__ == "__main__":
    main()

