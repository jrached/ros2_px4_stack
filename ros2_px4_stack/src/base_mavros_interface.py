#! /usr/bin/env python3

import rclpy
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
import math
from threading import Thread
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion, Vector3
from trajectory_msgs.msg import MultiDOFJointTrajectory, MultiDOFJointTrajectoryPoint
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
from mavros_msgs.srv import CommandBool, SetMode #, SetModeRequest, CommandBoolRequest
from sensor_msgs.msg import NavSatFix, Imu
from pymavlink import mavutil

from typing import List, Tuple

from six.moves import xrange


class BasicMavrosInterface(Node):
    def __init__(self, node_name="basic_mavros_interface"):
        super().__init__(node_name)

        qos_profile = QoSProfile(depth=10)
        qos_profile.durability = DurabilityPolicy.VOLATILE
        qos_profile.reliability = ReliabilityPolicy.BEST_EFFORT

        self.state = State()
        self.altitude = Altitude()
        self.extended_state = ExtendedState()
        self.global_position = NavSatFix()
        self.imu_data = Imu()
        self.home_position = HomePosition()
        self.local_position = PoseStamped()
        self.mission_wp = WaypointList()
        self.mav_type = None
        # self.received_trajectory_setpoint = None

        self.sub_topics_ready = {
            key: False
            for key in [
                "alt",
                "ext_state",
                "global_pos",
                "home_pos",
                "local_pos",
                "mission_wp",
                "state",
                "imu",
            ]
        }

        # ROS services
        self.get_param_srv = self.create_client(ParamGet, "mavros/param/get")
        self.set_param_srv = self.create_client(ParamSet, "mavros/param/set")
        self.set_arming_srv = self.create_client(CommandBool, "mavros/cmd/arming")
        self.set_mode_srv = self.create_client(SetMode, "mavros/set_mode")
        self.wp_clear_srv = self.create_client(WaypointClear, "mavros/mission/clear")
        self.wp_push_srv = self.create_client(WaypointPush, "mavros/mission/push")

        service_timeout = 1
        self.get_logger().info("waiting for ROS services")
        try:
            if self.get_param_srv.wait_for_service(service_timeout):
                self.get_logger().info('get_param service is up!')
            else:
                self.get_logger().info('get_param not available, proceeding without it...')

            if self.set_param_srv.wait_for_service(service_timeout):
                self.get_logger().info('set_param service is up!')
            else:
                self.get_logger().info('set_param not available, proceeding without it...')

            if self.set_arming_srv.wait_for_service(service_timeout):
                self.get_logger().info('set_arming service is up!')
            else:
                self.get_logger().info('set_arming not available, proceeding without it...')

            if self.set_mode_srv.wait_for_service(service_timeout):
                self.get_logger().info('set_mode service is up!')
            else:
                self.get_logger().info('set_mode not available, proceeding without it...')

            if self.wp_clear_srv.wait_for_service(service_timeout):
                self.get_logger().info('wp_clear service is up!')
            else:
                self.get_logger().info('wp_clear not available, proceeding without it...')

            if self.wp_push_srv.wait_for_service(service_timeout):
                self.get_logger().info('wp_push service is up!')
            else:
                self.get_logger().info('wp_push not available, proceeding without it...')

        except rclpy.ROSException as e:
            self.get_logger.error(e)
            raise e

        # ROS subscribers
        self.state_sub = self.create_subscription(State, "mavros/state", self.state_callback, qos_profile)
        self.alt_sub = self.create_subscription(Altitude,
            "mavros/altitude", self.altitude_callback, qos_profile
        )
        self.ext_state_sub = self.create_subscription(ExtendedState,
            "mavros/extended_state", self.extended_state_callback, qos_profile
        )
        self.global_pos_sub = self.create_subscription(NavSatFix,
            "mavros/global_position/global", self.global_position_callback, qos_profile
        )
        self.imu_data_sub = self.create_subscription(Imu,
            "mavros/imu/data", self.imu_data_callback, qos_profile
        )
        self.home_pos_sub = self.create_subscription(HomePosition,
            "mavros/home_position/home", self.home_position_callback, qos_profile
        )
        self.local_pos_sub = self.create_subscription(PoseStamped,
            "mavros/local_position/pose", self.local_position_callback, qos_profile
        )
        self.mission_wp_sub = self.create_subscription(WaypointList,
            "mavros/mission/waypoints", self.mission_wp_callback, qos_profile
        )
        

        # ROS publishers
        self.mavlink_pub = self.create_publisher(Mavlink, "mavlink/to", qos_profile)
        self.setpoint_position_pub = self.create_publisher(PoseStamped,
            "mavros/setpoint_position/local", 
            qos_profile
        )
        self.setpoint_traj_pub = self.create_publisher(MultiDOFJointTrajectory, 
        "mavros/setpoint_trajectory/local", 
        qos_profile
        )

        # need to simulate heartbeat to prevent datalink loss detection
        self.hb_mav_msg = mavutil.mavlink.MAVLink_heartbeat_message(
            mavutil.mavlink.MAV_TYPE_GCS, 0, 0, 0, 0, 0
        )
        self.hb_mav_msg.pack(mavutil.mavlink.MAVLink("", 2, 1))
        self.hb_ros_msg = mavlink.convert_to_rosmsg(self.hb_mav_msg)
        self.hb_thread = Thread(target=self.send_heartbeat, args=())
        self.hb_thread.daemon = True
        self.hb_thread.start()

        # TODO: shouldn't need to send heartbeat manually, should be doable from mavros

    def send_heartbeat(self):
        rate = self.create_rate(2)  # Hz
        while rclpy.ok():
            self.mavlink_pub.publish(self.hb_ros_msg)
            try:  # prevent garbage in console output when thread is killed
                rate.sleep()
            except ExternalShutdownException:
                pass


    def altitude_callback(self, data):
        self.altitude = data

        # amsl has been observed to be nan while other fields are valid
        if not self.sub_topics_ready["alt"] and not math.isnan(data.amsl):
            self.sub_topics_ready["alt"] = True

    def extended_state_callback(self, data):
        if self.extended_state.vtol_state != data.vtol_state:
            self.get_logger.info(
                "VTOL state changed from {0} to {1}".format(
                    mavutil.mavlink.enums["MAV_VTOL_STATE"][
                        self.extended_state.vtol_state
                    ].name,
                    mavutil.mavlink.enums["MAV_VTOL_STATE"][data.vtol_state].name,
                )
            )

        if self.extended_state.landed_state != data.landed_state:
            self.get_logger().info(
                "landed state changed from {0} to {1}".format(
                    mavutil.mavlink.enums["MAV_LANDED_STATE"][
                        self.extended_state.landed_state
                    ].name,
                    mavutil.mavlink.enums["MAV_LANDED_STATE"][data.landed_state].name,
                )
            )

        self.extended_state = data

        if not self.sub_topics_ready["ext_state"]:
            self.sub_topics_ready["ext_state"] = True

    def global_position_callback(self, data):
        self.global_position = data

        if not self.sub_topics_ready["global_pos"]:
            self.sub_topics_ready["global_pos"] = True

    def imu_data_callback(self, data):
        self.imu_data = data

        if not self.sub_topics_ready["imu"]:
            self.sub_topics_ready["imu"] = True

    def home_position_callback(self, data):
        self.home_position = data

        if not self.sub_topics_ready["home_pos"]:
            self.sub_topics_ready["home_pos"] = True

    def local_position_callback(self, data):
        self.local_position = data

        if not self.sub_topics_ready["local_pos"]:
            self.sub_topics_ready["local_pos"] = True


    def mission_wp_callback(self, data):
        if self.mission_wp.current_seq != data.current_seq:
            self.get_logger().info(
                "current mission waypoint sequence updated: {0}".format(
                    data.current_seq
                )
            )

        self.mission_wp = data

        if not self.sub_topics_ready["mission_wp"]:
            self.sub_topics_ready["mission_wp"] = True

    def state_callback(self, data):
        if self.state.armed != data.armed:
            self.get_logger().info(
                "armed state changed from {0} to {1}".format(
                    self.state.armed, data.armed
                )
            )

        if self.state.connected != data.connected:
            self.get_logger().info(
                "connected changed from {0} to {1}".format(
                    self.state.connected, data.connected
                )
            )

        if self.state.mode != data.mode:
            self.get_logger().info(
                "mode changed from {0} to {1}".format(self.state.mode, data.mode)
            )

        if self.state.system_status != data.system_status:
            self.get_logger().info(
                "system_status changed from {0} to {1}".format(
                    mavutil.mavlink.enums["MAV_STATE"][self.state.system_status].name,
                    mavutil.mavlink.enums["MAV_STATE"][data.system_status].name,
                )
            )

        self.state = data

        # mavros publishes a disconnected state message on init
        if not self.sub_topics_ready["state"] and data.connected:
            self.sub_topics_ready["state"] = True

    #
    # Useful properties
    #
    @property
    def is_armed(self):
        return self.state.armed

    @property
    def is_connected(self):
        return self.state.connected

    #
    # Helper methods
    #
    def set_arm(self, arm, timeout):
        """arm: True to arm or False to disarm, timeout(int): seconds"""
        self.get_logger().info("setting FCU arm: {0}".format(arm))
        old_arm = self.state.armed
        loop_freq = 1  # Hz
        rate = self.create_rate(loop_freq)
        arm_set = False
        for i in xrange(timeout * loop_freq):
            if self.state.armed == arm:
                arm_set = True
                self.get_logger().info(
                    "set arm success | seconds: {0} of {1}".format(
                        i / loop_freq, timeout
                    )
                )
                break
            else:
                try:
                    res = self.set_arming_srv.call_async(arm)
                    rclpy.spin_until_future_complete(self, res)
                    if res.result() is None:
                        self.get_logger.error(
                            "failed to send arm command! Success: {0} and Result: {1}".format(
                                res.success, res.result
                            )
                        )
                except rclpy.ServiceException as e:
                    self.get_logger.error(e)

            try:
                rate.sleep()
            except rclpy.ROSException as e:
                self.fail(e)

        assert (
            arm_set
        ), "failed to set arm | new arm: {0}, old arm: {1} | timeout(seconds): {2}".format(
            arm, old_arm, timeout
        )

    def set_mode(self, mode, timeout):
        """mode: PX4 mode string, timeout(int): seconds"""
        self.get_logger().info("setting FCU mode: {0}".format(mode))
        old_mode = self.state.mode
        loop_freq = 1  # Hz
        rate = self.create_rate(loop_freq)
        mode_set = False
        for i in xrange(timeout * loop_freq):
            if self.state.mode == mode:
                mode_set = True
                self.get_logger().info(
                    "set mode success | seconds: {0} of {1}".format(
                        i / loop_freq, timeout
                    )
                )
                break
            else:
                try:
                    res = self.set_mode_srv.call_async(0, mode)  # 0 is custom mode
                    rclpy.spin_until_future_complete(self, res)
                    if res.result() is None: #mght need to change back to if not res.mode: 
                        self.get_logger.error("failed to send mode command")
                except rclpy.ServiceException as e:
                    self.get_logger.error(e)

            try:
                rate.sleep()
            except rclpy.ROSException as e:
                self.fail(e)

        assert (
            mode_set
        ), "failed to set mode | new mode: {0}, old mode: {1} | timeout(seconds): {2}".format(
            mode, old_mode, timeout
        )

    def set_param(self, param_id, param_value, timeout):
        """param: PX4 param string, ParamValue, timeout(int): seconds"""
        if param_value.integer != 0:
            value = param_value.integer
        else:
            value = param_value.real
        self.get_logger().info(
            "setting PX4 parameter: {0} with value {1}".format(param_id, value)
        )
        loop_freq = 1  # Hz
        rate = self.create_rate(loop_freq)
        param_set = False
        for i in xrange(timeout * loop_freq):
            try:
                res = self.set_param_srv.call_async(param_id, param_value)
                rclpy.spin_until_future_complete(self, res)               
                if res.result() is not None:
                    self.get_logger().info(
                        "param {0} set to {1} | seconds: {2} of {3}".format(
                            param_id, value, i / loop_freq, timeout
                        )
                    )
                break
            except rclpy.ServiceException as e:
                self.get_logger.error(e)

            try:
                rate.sleep()
            except rclpy.ROSException as e:
                self.fail(e)

        assert (
            res.success
        ), "failed to set param | param_id: {0}, param_value: {1} | timeout(seconds): {2}".format(
            param_id, value, timeout
        )

    def set_local_setpoint(self, setpoint: PoseStamped):
        """Sets a local setpoint

        Args:
            setpoint (PoseStamped): the setpoint to send
        """

        # send the setpoint a few times to ensure it's received
        send_frequency = 10  # Hz
        num_sends = 5
        rate = self.create_rate(send_frequency)
        for i in range(num_sends):
            self.setpoint_position_pub.publish(setpoint)
            rate.sleep()
            # rospy.sleep(1 / send_frequency)

    def setpoint_reached(self, setpoint: PoseStamped, tolerance: float = 0.1) -> bool:
        """Checks if a setpoint has been reached

        Args:
            setpoint (PoseStamped): the setpoint to check
            tolerance (float, optional): the tolerance to use for checking. Defaults to 0.1.

        Returns:
            bool: True if the setpoint has been reached, False otherwise
        """
        x_reached = (
            abs(self.local_position.pose.position.x - setpoint.pose.position.x)
            <= tolerance
        )
        y_reached = (
            abs(self.local_position.pose.position.y - setpoint.pose.position.y)
            <= tolerance
        )
        z_reached = (
            abs(self.local_position.pose.position.z - setpoint.pose.position.z)
            <= tolerance
        )
        return x_reached and y_reached and z_reached


    def traj_point_reached(self, traj: MultiDOFJointTrajectory, tolerance: float = 0.1) -> bool:
        point = traj.points[0]
        transform = point.transforms[0]

        x_reached = (
            abs(self.local_position.pose.position.x - transform.translation.x)
            <= tolerance
        )
        y_reached = (
            abs(self.local_position.pose.position.y - transform.translation.y)
            <= tolerance
        )
        z_reached = (
            abs(self.local_position.pose.position.z - transform.translation.z)
            <= tolerance
        )
        return x_reached and y_reached and z_reached 

    def wait_for_landed_state(self, desired_landed_state, timeout, index):
        self.get_logger().info(
            "waiting for landed state | state: {0}, index: {1}".format(
                mavutil.mavlink.enums["MAV_LANDED_STATE"][desired_landed_state].name,
                index,
            )
        )
        loop_freq = 10  # Hz
        rate = self.create_rate(loop_freq)
        landed_state_confirmed = False
        for i in xrange(timeout * loop_freq):
            if self.extended_state.landed_state == desired_landed_state:
                landed_state_confirmed = True
                self.get_logger().info(
                    "landed state confirmed | seconds: {0} of {1}".format(
                        i / loop_freq, timeout
                    )
                )
                break

            try:
                rate.sleep()
            except rclpy.ROSException as e:
                self.fail(e)

        assert (
            landed_state_confirmed
        ), "landed state not detected | desired: {0}, current: {1} | index: {2}, timeout(seconds): {3}".format(
            mavutil.mavlink.enums["MAV_LANDED_STATE"][desired_landed_state].name,
            mavutil.mavlink.enums["MAV_LANDED_STATE"][
                self.extended_state.landed_state
            ].name,
            index,
            timeout,
        )

    def wait_for_vtol_state(self, transition, timeout, index):
        """Wait for VTOL transition, timeout(int): seconds"""
        self.get_logger().info(
            "waiting for VTOL transition | transition: {0}, index: {1}".format(
                mavutil.mavlink.enums["MAV_VTOL_STATE"][transition].name, index
            )
        )
        loop_freq = 10  # Hz
        rate = self.create_rate(loop_freq)
        transitioned = False
        for i in xrange(timeout * loop_freq):
            if transition == self.extended_state.vtol_state:
                self.get_logger().info(
                    "transitioned | seconds: {0} of {1}".format(i / loop_freq, timeout)
                )
                transitioned = True
                break

            try:
                rate.sleep()
            except rclpy.ROSException as e:
                self.fail(e)

        assert (
            transitioned
        ), "transition not detected | desired: {0}, current: {1} | index: {2} timeout(seconds): {3}".format(
            mavutil.mavlink.enums["MAV_VTOL_STATE"][transition].name,
            mavutil.mavlink.enums["MAV_VTOL_STATE"][
                self.extended_state.vtol_state
            ].name,
            index,
            timeout,
        )

    def clear_wps(self, timeout):
        """timeout(int): seconds"""
        loop_freq = 1  # Hz
        rate = self.create_rate(loop_freq)
        wps_cleared = False
        for i in xrange(timeout * loop_freq):
            if not self.mission_wp.waypoints:
                wps_cleared = True
                self.get_logger().info(
                    "clear waypoints success | seconds: {0} of {1}".format(
                        i / loop_freq, timeout
                    )
                )
                break
            else:
                try:
                    res = self.wp_clear_srv.call_async()
                    rclpy.spin_until_future_complete(self, res)
                    if res.result() is None:
                        self.get_logger.error("failed to send waypoint clear command")
                except rclpy.ServiceException as e:
                    self.get_logger.error(e)

            try:
                rate.sleep()
            except rclpy.ROSException as e:
                self.fail(e)

        assert wps_cleared, "failed to clear waypoints | timeout(seconds): {0}".format(
            timeout
        )

    def send_wps(self, waypoints: List[Waypoint], timeout: float):
        """
        Args:
            waypoints (List[Waypoint]): list of waypoints to send to FCU
            timeout (int): seconds

            WaypointPush
            # Send waypoints to device
            #
            #  :start_index: will define a partial waypoint update. Set to 0 for full update
            #
            # Returns success status and transferred count

            uint16 start_index
            mavros_msgs/Waypoint[] waypoints
            ---
            bool success
            uint32 wp_transfered
        """

        assert isinstance(waypoints, list), "waypoints must be a list"
        assert len(waypoints) > 0, "waypoints list must not be empty"
        assert all(
            isinstance(wp, Waypoint) for wp in waypoints
        ), "waypoints must be a list of Waypoint objects"

        self.get_logger().info("sending mission waypoints")

        if self.mission_wp.waypoints:
            self.get_logger().info("FCU already has mission waypoints")

        loop_freq = 1  # Hz
        rate = self.create_rate(loop_freq)
        wps_sent = None
        wps_verified = False
        for i in xrange(timeout * loop_freq):
            if wps_sent is None:
                try:
                    res = self.wp_push_srv.call_async(start_index=0, waypoints=waypoints)
                    rclpy.spin_until_future_complete(self, res)
                    wps_sent = res.result()
                    if wps_sent is not None:
                        self.get_logger().info("waypoints successfully transferred")
                    else:
                        self.get_logger.error("failed to send waypoints")
                except rclpy.ServiceException as e:
                    self.get_logger.error(e)
            else:
                if len(waypoints) == len(self.mission_wp.waypoints):
                    self.get_logger().info(
                        "number of waypoints transferred: {0}".format(len(waypoints))
                    )
                    wps_verified = True
                else:
                    self.get_logger().info(
                        "number of waypoints transferred: {0} of {1}".format(
                            len(self.mission_wp.waypoints), len(waypoints)
                        )
                    )

            if (wps_sent is not None) and wps_verified:
                self.get_logger().info(
                    "send waypoints success | seconds: {0} of {1}".format(
                        i / loop_freq, timeout
                    )
                )
                break

            try:
                rate.sleep()
            except rclpy.ROSException as e:
                self.fail(e)

        assert (
            (wps_sent is not None) and wps_verified
        ), "mission could not be transferred and verified | timeout(seconds): {0}".format(
            timeout
        )

    def wait_for_mav_type(self, timeout):
        """Wait for MAV_TYPE parameter, timeout(int): seconds"""
        self.get_logger().info("waiting for MAV_TYPE")
        loop_freq = 1  # Hz
        rate = self.create_rate(loop_freq)
        res = False
        for i in xrange(timeout * loop_freq):
            try:
                res = self.get_param_srv.call_async("MAV_TYPE")
                rclpy.spin_until_future_complete(self, res)
                if res.result() is not None:
                    self.mav_type = res.value.integer
                    self.get_logger().info(
                        "MAV_TYPE received | type: {0} | seconds: {1} of {2}".format(
                            mavutil.mavlink.enums["MAV_TYPE"][self.mav_type].name,
                            i / loop_freq,
                            timeout,
                        )
                    )
                    break
            except rclpy.ServiceException as e:
                self.get_logger.error(e)

            try:
                rate.sleep()
            except rclpy.ROSException as e:
                self.fail(e)

        assert res.result() is not None, "MAV_TYPE param get failed | timeout(seconds): {0}".format(
            timeout
        )

    def log_topic_vars(self):
        """log the state of topic variables"""
        self.get_logger().info("========================")
        self.get_logger().info("===== topic values =====")
        self.get_logger().info("========================")
        self.get_logger().info("altitude:\n{}".format(self.altitude))
        self.get_logger().info("========================")
        self.get_logger().info("extended_state:\n{}".format(self.extended_state))
        self.get_logger().info("========================")
        self.get_logger().info("global_position:\n{}".format(self.global_position))
        self.get_logger().info("========================")
        self.get_logger().info("home_position:\n{}".format(self.home_position))
        self.get_logger().info("========================")
        self.get_logger().info("local_position:\n{}".format(self.local_position))
        self.get_logger().info("========================")
        self.get_logger().info("mission_wp:\n{}".format(self.mission_wp))
        self.get_logger().info("========================")
        self.get_logger().info("state:\n{}".format(self.state))
        self.get_logger().info("========================")


def main():
    rclpy.init()
    node = BasicMavrosInterface()
    rclpy.spin(node)
    

if __name__ == "__main__":
    main()