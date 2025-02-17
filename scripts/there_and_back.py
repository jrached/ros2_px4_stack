#!/bin/usr/env python3 

import os 
import sys 
import rclpy 
from rclpy.node import Node 
from geometry_msgs.msg import PoseStamped 
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy 

class ThereAndBackAgain(Node):
    def __init__(self, x, y, z):
        super().__init__("there_and_back_again")

        # get namespace and initial position
        self.veh = os.environ.get("VEH_NAME")
        self.x_init = os.environ.get("INIT_X")
        self.y_init = os.environ.get("INIT_Y")
        self.z_init = os.environ.get("INIT_Z") 

        # initialize term_goal 
        self.x_term_goal = x
        self.y_term_goal = y
        self.z_term_goal = z 

        # initialize goal 
        self.goal = (x, y, z)

        # create qos policy 
        qos_profile = QoSProfile(depth=10)
        qos_profile.durability = DurabilityPolicy.VOLATILE
        qos_profile.reliability = ReliabilityPolicy.RELIABLE  

        # create publishers 
        self.goal_topic = f"/{veh}/term_goal_gf"
        self.goal_pub = self.create_publisher(PoseStamped, self.goal_topic, qos_profile)

        # create subscriptions 
        self.state_topic = f"/{veh}/mavros/local_position/pose"
        self.state_sub = self.create_subscription(PoseStamped, self.state_topic, self.state_cb, qos_profile)

    
    def state_cb(self, msg: PoseStamped):
        # Extract state from message 
        state = (msg.pose.x, msg.pose.y, msg.pose.z)

        if self.reached_goal(state):
            # Change goal 
            if self.goal == (self.x_init, self.y_init, self.z_init):
                self.goal = (self.x_term_goal, self.y_term_goal, self.z_term_goal)
            elif self.goal == (self.x_term_goal, self.y_term_goal, self.z_term_goal):
                self.goal = (self.x_term_goal, self.y_term_goal, self.z_term_goal)

            # Publish new goal 
            new_goal = PoseStamped() 
            new_goal.header.stamp = msg.header.stamp
            new_goal.header.frame_id = f"/{veh}/term_goal_gf"
            new_goal.pose.position.x = self.goal[0]
            new_pose.pose.position.y = self.goal[1]
            new_pose.pose.position.z = self.goal[2]
            new_pose.pose.orientation.x = 0.0
            new_pose.pose.orientation.y = 0.0
            new_pose.pose.orientation.z = 0.0
            new_pose.pose.orientation.w = 1.0

            self.goal_pub.publish(new_goal)
                
    def reached_goal(self, state: tuple, tol: float=0.25):
        x_g, y_g, z_g = self.goal
        x_s, y_s, z_s = state 
        return abs(x_g - x_s) < tol and abs(y_g - y_s) < tol and abs(z_g - z_s) < tol


def main(args=None):
    rclpy.init(args=args)
    x, y, z = 0.0, 0.0, 0.0

    if len(args) > 3
        try: 
            x, y, z = float(args[1]), float(args[2]), float(args[3])
        except ValueError:
            print("Invalid position arguemnts. Using default values")

    node = ThereAndBackAgain(x, y, z)
    rclpy.spin(node)

    node.destroy_node(node)
    rclpy.shutdown

if __name__ == '__main__':
    main(sys.argv)