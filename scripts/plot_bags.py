#!/bin/usr/env python3 

import rclpy
from rclpy.node import Node 

from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped

from matplotlib import pyplot as plt 

class PlotPIDResponse(Node):
    def __init__(self):
        super().__init__("plot_pid_response")

        # Variables 
        self.bag2_is_published = False

        # Make subscriptions
        bag1_type = None #TODO
        bag1_topic = ""
        self.bag1_subscription = self.create_subscription(bag1_type, bag1_topic, self.bag1_cb, 10)

        bag2_type = None #TODO
        bag2_topic = ""
        self.bag2_subscription = self.create_subscription(bag2_type, bag2_topic, self.bag2_cb, 10)

        plot_type = String()
        plot_topic = "/plot_bags"
        self.plot_subscription = self.create_subscription(plot_type, plot_topic, self.plot_cb, 10)

        # Declare data arrays
        self.setpoint_timestamp = []
        self.setpoint_roll = []
        self.setpoint_pitch = []

        self.measured_timestamp = []
        self.measured_roll = []
        self.measured_pitch = []

    def bag1_cb(self, msg):
        timestamp = msg.header.stamp
        att = msg.pose.orientation
        qx = att.x
        qy = att.y
        qz = att.z
        qw = att.w

        roll, pitch, yaw = euler_from_quaternion(qx, qy, qz, qw)
        
        #TODO define bag_2_publisher_count (there's a method for it)
        self.bag2_is_published = len(bag_2_publisher_count) > 0 

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


#TODO: write function 
def euler_from_quaternion(qx, qy, qz, qw):
    pass


def main(args=None):
    rclpy.init(args=args)
    plot_bags = PlotPIDResponse()
    rclpy.spin(plot_bags)


if __name__ == '__main__':
    main()

    
        