#! /usr/bin/env python3

import rclpy
from rclpy.node import Node
import math
from threading import Thread
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
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
    #SetModeRequest,
    WaypointClear,
    WaypointPush,
)
from mavros_msgs.srv import CommandBool, SetMode #SetModeRequest, CommandBoolRequest
from sensor_msgs.msg import NavSatFix, Imu
from pymavlink import mavutil

from typing import List, Tuple

from six.moves import xrange

LOCAL_NAVIGATION = 0  # x/y/z relative to home position
GLOBAL_NAVIGATION = 1  # lat/lon/alt
NAVIGATION_MODES = [LOCAL_NAVIGATION, GLOBAL_NAVIGATION]


from .offboard_node import OffboardPathFollower


class SquarePathFollower(OffboardPathFollower):

    def __init__(
        self,
        node_name: str = "square_path_follower",
        navigation_mode: int = LOCAL_NAVIGATION,
    ):
        super().__init__(node_name=node_name, navigation_mode=navigation_mode)

    def track_square(self, side_length: float, altitude: float):
        """Generate a square path and track it

        Args:
            side_length (float): the length of each side of the square
            altitude (float): the altitude to fly at
        """

        assert self.navigation_mode == LOCAL_NAVIGATION, (
            f"Invalid navigation mode: {self.navigation_mode}. "
            f"Only local navigation is supported for this method"
        )


        self.get_logger().info(
            f"Tracking square of side length {side_length} at altitude {altitude}"
        )

        x_origin = 0.0
        y_origin = -2.3
        vertices = [
            (x_origin + side_length, y_origin + 0, altitude),
            (x_origin + side_length, y_origin + side_length, altitude),
            (x_origin + 0, y_origin + side_length, altitude),
            (x_origin + 0, y_origin + 0, altitude),
        ]

        setpoints = self._pack_into_setpoints(vertices)
        self.track_setpoints(setpoints, altitude)

def main():
    rclpy.init()
    node_name = "square_path_follower"
    path_follower = SquarePathFollower(
        node_name=node_name, navigation_mode=LOCAL_NAVIGATION
    )

    path_follower.track_square(side_length=2.5, altitude=1.8)

if __name__ == "__main__":
    main()