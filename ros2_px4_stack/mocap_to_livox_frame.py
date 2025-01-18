#!/usr/bin/env python3

import os 
import rclpy 
from rclpy.node import Node 
from geometry_msgs.msg import PoseStamped, TransformStamped

from tf2_ros import Buffer, TransformListener, TransformBroadcaster
from tf2_geometry_msgs import do_transform_pose

veh = os.environ.get("VEH_NAME")

class MocapToLivoxFrame(Node):
    def __init__(self):
        super().__init__('mocap_to_livox_frame')

        # Make subscription for global frame goal command
        self.gf_goal_sub = self.create_subscription(PoseStamped, "term_goal_gf", self.goal_cb, 10)

        # Make publisher for agent frame command 
        self.af_goal_pub = self.create_publisher(PoseStamped, "term_goal", 10)

        # Make transform listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Class variables
        self.global_to_agent_tf = None 

    def get_gf_to_af_tf(self):
        # Get static transform between livox initial position and mocap origin
        while not self.tf_buffer.can_transform(f"{veh}/init_pose", "world_mocap", self.get_clock().now()):
            self.get_logger().info(f"Waiting for transform")
            rclpy.spin_once(self, timeout_sec=1.0)

        return self.tf_buffer.lookup_transform(f"{veh}/init_pose", "world_mocap", self.get_clock().now())

    def goal_cb(self, msg):
        """
        Transforms trajectory terminal goal command in global frame to agent frame. 

        Input: Terminal goal of trajectory in the global frame, PoseStamped

        Output: Terminal goal of trajectory in the agent frame, PoseStamped
        """
        
        if not self.global_to_agent_tf:
            self.global_to_agent_tf = self.get_gf_to_af_tf()
        
        af_goal = do_transform_pose(msg.pose, self.global_to_agent_tf)

        af_goal_stamped = PoseStamped()
        af_goal_stamped.header.stamp = msg.header.stamp 
        af_goal_stamped.header.frame_id = msg.header.frame_id
        af_goal_stamped.pose = af_goal 

        self.af_goal_pub.publish(af_goal_stamped)

        tfs = TransformStamped()
        tfs.header.stamp = msg.header.stamp 
        tfs.header.frame_id = f"{veh}/init_pose"
        tfs.child_frame_id = f"{veh}/pose_af"
        tfs.transform.translation.x = af_goal.position.x  
        tfs.transform.translation.y = af_goal.position.y  
        tfs.transform.translation.z = af_goal.position.z  
        tfs.transform.rotation = af_goal.orientation 

        broadcaster.sendTransform(tfs) 


def main(args=None):
    rclpy.init(args=args)
    node = MocapToLivoxFrame()

    global broadcaster
    broadcaster = TransformBroadcaster(node)
    try: 
        rclpy.spin(node)
    except KeyboardInterrupt: 
        pass

    rclpy.shutdown()

if __name__ == '__main__':
    main()