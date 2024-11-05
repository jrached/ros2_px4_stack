#!/bin/usr/env python3 

import rclpy
from rclpy.node import Node 

from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped

from matplotlib import pyplot as plt 

from transforms3d import quat2euler

import pandas as pd

class PlotPIDResponse(Node):
    def __init__(self):
        super().__init__("plot_pid_response")

        # Variables 
        self.bag2_is_published = False

        # Make subscriptions
        bag1_type = PoseStamped #TODO verify it is PoseStamped
        bag1_topic = "/mavros/local_position/pose"
        self.bag1_subscription = self.create_subscription(bag1_type, bag1_topic, self.bag1_cb, 10)

        self.bag2_type = None #TODO
        self.bag2_topic = "/mavros/setpoint_raw/target_attitude"
        self.bag2_subscription = self.create_subscription(self.bag2_type, self.bag2_topic, self.bag2_cb, 10)

        self.plot_type = String
        self.plot_topic = "/plot_bags"
        self.plot_subscription = self.create_subscription(self.plot_type, self.plot_topic, self.plot_cb, 10)

        # Declare data arrays
        self.setpoint_timestamp = []
        self.setpoint_roll = []
        self.setpoint_pitch = []

        self.measured_timestamp = []
        self.measured_roll = []
        self.measured_pitch = []

        # Create datasets
        self.df1 = None
        self.df2 = None 

    def bag1_cb(self, msg):
        timestamp = msg.header.stamp
        att = msg.pose.orientation
        qx = att.x
        qy = att.y
        qz = att.z
        qw = att.w

        roll, pitch, yaw = euler_from_quaternion(qx, qy, qz, qw)
        
        bag2_publisher_count = len(self.get_publisher_info_by_topic(self.bag2_topic))

        self.bag2_is_published = bag_2_publisher_count > 0 

        if self.bag2_is_published:
            self.setpoint_timestamp.append(timestamp)
            self.setpoint_roll.append(roll)
            self.setpoint_pitch.append(pitch)

            

    
    def bag2_cb(self, msg):
        timestamp = msg.hader.stamp
        att = msg.pose.orientation
        qx = att.x
        qy = att.y
        qz = att.z
        qw = att.w

        roll, pitch, yaw = euler_from_quaternion(qx, qy, qz, qw)



        self.measured_timestamp.append(timestamp)
        self.measured_roll.append(roll)
        self.measured_pitch.append(pitch)

    
    def plot_cb(self):
        #TODO: Make plots

        while rclpy.ok():
            #do nothing. Just wait until I'm done looking at my plots.
            continue 


def euler_from_quaternion(qx, qy, qz, qw):
    return quat2euler([qw, qx, qy, qz], axes='sxyz')


def main(args=None):
    rclpy.init(args=args)
    plot_bags = PlotPIDResponse()
    rclpy.spin(plot_bags)


if __name__ == '__main__':
    main()

    
        