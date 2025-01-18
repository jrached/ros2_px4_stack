#!/usr/bin/env python3

import os 
import rclpy 
from rclpy.node import Node 
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from geometry_msgs.msg import PoseStamped, TwistStamped, TransformStamped, Vector3, Vector3Stamped  
from nav_msgs.msg import Odometry 
import math

from tf2_ros import Buffer, TransformListener, TransformBroadcaster
from tf2_geometry_msgs import do_transform_pose, do_transform_vector3

import numpy as np 
from dynus_interfaces.msg import Goal  

veh = os.environ.get("VEH_NAME")

class MocapToLivoxFrame(Node):
    def __init__(self):
        """
        TODO: Consult Kota. I think for now you can assume quasi-static behavior from the drone and say that roll and
        pitch are equal to zero. The actual solution is to obtain the roll and pitch encoded in the position, velocity, and 
        acceleration commands 
        """
        super().__init__('mocap_to_livox_frame')

        qos_profile = QoSProfile(depth=10)
        qos_profile.durability = DurabilityPolicy.VOLATILE
        qos_profile.reliability = ReliabilityPolicy.BEST_EFFORT

        # Make subscription to dlio odom topic
        # self.dlio_sub = self.create_subscription(Odometry, f"{veh}/dlio/odom_node/odom", self.dlio_cb, 10)
        # self.dlio_sub = self.create_subscription(PoseStamped, f"{veh}/dlio/odom_node/pose", self.dlio_cb, 10)

        # Make global frame publishers for mavros pose and twist 
        self.mavros_pose_pub = self.create_publisher(PoseStamped, f"/{veh}/pose_gf", 10)
        self.mavros_twist_pub = self.create_publisher(TwistStamped, f"{veh}/twist_gf", 10)

        # Make global frame publisher for dynus pose and twist 
        #TODO: Check dynus code for how this is represeneted 
        #I.e., does it subscribe to odom? Or does it subscribe to pose only? 

        # Make transform listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        ### For Debugging ### 
        # # Make mocap subscription
        self.pose_subscription = self.create_subscription(PoseStamped, f"/{veh}/world", self.dlio_cb, 10)
        # self.twist_subscription = self.create_subscription(TwistStamped, f"/{veh}/mocap/twist", self.twist_cb, 10) 
        # Make dynus goal topic subscription 
        # self.dynus_subscription = self.create_subscription(Goal, f"/{veh}/goal", self.transform_cb, 10)
        
        # Define velocity publisher 
        # self.vel_publisher = self.create_publisher(TwistStamped, f"/{veh}/livox_vel_estimate", 10)
        # Make dynus goal topic publisher 
        # self.dynus_publisher = self.create_publisher(Goal, f"/{veh}/agent_frame_goal", 10)
         # Make subscription to /term_goal_gf (terminal goal in global frame) 
        # self.goal_subscription = self.create_subscription(PoseStamped, f"/{veh}/term_goal_gf", self.goal_cb, 10)

        # Make publisher to dynus term_goal topic 
        # self.goal_publisher = self.create_publisher(PoseStamped, f"/{veh}/term_goal", 10)

        # Make timer for transform broadcaster
        # pub_freq = 25 # Hz 
        # timer_period = 1 / pub_freq 
        # self.timer = self.create_timer(timer_period, self.repub_transform)

        # Class variables to store mocap pose and twist
        self.mocap_pose = None
        self.mocap_twist = None 


    def get_af_to_gf_tf(self):
        # Get static transform between mocap origin and livox initial position 
        while not self.tf_buffer.can_transform( "world_mocap", f"{veh}/init_pose", self.get_clock().now()):
            self.get_logger().info(f"Waiting for transform")
            rclpy.spin_once(self, timeout_sec=1.0)

        return self.tf_buffer.lookup_transform( "world_mocap", f"{veh}/init_pose", self.get_clock().now())

    def transform_pose(self, pose):
        """
        Input: agent frame pose, Pose
        Output: global frame pose, Pose
        """

        tf = self.get_af_to_gf_tf() 
        new_pose = do_transform_pose(pose, tf)

        return new_pose 

    def transform_twist(self, twist):
        """
        Input: agent frame twist, Twist
        Output: global frame twist, TwistStamped
        """
        tf= self.get_af_to_gf_tf()

        # Constuct rot_only_tf from tf here
        rot_only_tf = TransformStamped()
        rot_only_tf.header = tf.header 
        rot_only_tf.child_frame_id = tf.child_frame_id
        rot_only_tf.transform.translation.x = 0.0
        rot_only_tf.transform.translation.y = 0.0
        rot_only_tf.transform.translation.z = 0.0
        rot_only_tf.transform.rotation = tf.transform.rotation 

        # Construct velocity vector3stamped from TwistStamped 
        vel = Vector3Stamped()
        vel.header.stamp = self.get_clock().now().to_msg()
        vel.header.frame_id = "vel"
        vel.vector = twist.linear 

        new_vel = do_transform_vector3(vel, rot_only_tf).vector

        # Construct velocity vector3stamped from TwistStamped 
        rate = Vector3Stamped()
        rate.header.stamp = self.get_clock().now().to_msg()
        rate.header.frame_id = "rate"
        rate.vector = twist.angular 

        new_rate = do_transform_vector3(rate, rot_only_tf).vector 

        # Construct TwistStamped from linear and angular velocity vector3 
        new_ts = TwistStamped() 
        new_ts.header.stamp = self.get_clock().now().to_msg() 
        new_ts.header.frame_id = f"{veh}/twist_gf"
        new_ts.twist.linear = new_vel 
        new_ts.twist.angular = new_rate 

        return new_ts


    def dlio_cb(self, msg):
        """
        Transforms dlio pose and twist estimate from the agent frame to the global frame and publishes 
        the global frame estimates to mavros and dynus. 

        Input: DLIO pose and twist estimates in agent frame, Odometry. 
               Odometry contains PoseWithCovariance
                                 TwistWithCovariance

        Output: Mavros pose in global frame, PoseStamped
                Mavros twist in global frame, TwistStamped
                Dynus pose in global frame, PoseStamped
                Dynus twist in global frame, TwistStamped
        """
        #TODO: Implement 

        # 1. Transform pose estimate
        # new_pose = self.transform_pose(msg.pose.pose)
        new_pose = self.transform_pose(msg.pose)
        new_ps = PoseStamped() 
        new_ps.header.stamp = msg.header.stamp
        new_ps.header.frame_id = "pose_gf"
        new_ps.pose = new_pose 
        # 2. Transform twist estimate 
        # new_twist = self.transform_twist(msg.twist.twist) 
        # 3. Publish pose and twist messages for mavros repub
        self.mavros_pose_pub.publish(new_ps)
        # self.mavros_twist_pub.publish(new_twist)


        tfs = TransformStamped()
        tfs.header.stamp = msg.header.stamp 
        tfs.header.frame_id = f"world_mocap"
        tfs.child_frame_id = f"{veh}/pose_gf"
        tfs.transform.translation.x = new_ps.pose.position.x  
        tfs.transform.translation.y = new_ps.pose.position.y  
        tfs.transform.translation.z = new_ps.pose.position.z  
        tfs.transform.rotation = new_ps.pose.orientation 

        broadcaster.sendTransform(tfs) 

        # 4. Populate dynus messages 
        # 5. Publish for messages to dynus topics 



########################
### Helper Functions ###
########################
def quaternion_from_euler(roll, pitch, yaw):
    """
    Converts euler roll, pitch, yaw to quaternion (w in last place)
    quat = [x, y, z, w]
    Bellow should be replaced when porting for ROS 2 Python tf_conversions is done.
    """
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    q = [0] * 4

    q[0] = cy * cp * sr - sy * sp * cr
    q[1] = sy * cp * sr + cy * sp * cr
    q[2] = sy * cp * cr - cy * sp * sr
    q[3] = cy * cp * cr + sy * sp * sr

    return q

def euler_from_quat(quaternion):
        """
        Converts quaternion (w in last place) to euler roll, pitch, yaw
        quaternion = [x, y, z, w]
        Bellow should be replaced when porting for ROS 2 Python tf_conversions is done.
        """
        x = quaternion.x
        y = quaternion.y
        z = quaternion.z
        w = quaternion.w

        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = np.arctan2(sinr_cosp, cosr_cosp)

        sinp = 2 * (w * y - z * x)
        sinp = np.clip(sinp, -1.0, 1.0)  # Clamping to handle numerical errors
        pitch = np.arcsin(sinp)

        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = np.arctan2(siny_cosp, cosy_cosp)

        return roll, pitch, yaw

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



######## OLD #################################################

    # def get_gf_to_af_tf(self):
    #     # Get static transform between livox initial position and mocap origin
    #     while not self.tf_buffer.can_transform(f"{veh}/init_pose", "world_mocap", self.get_clock().now()):
    #         self.get_logger().info(f"Waiting for transform")
    #         rclpy.spin_once(self, timeout_sec=1.0)

    #     return self.tf_buffer.lookup_transform(f"{veh}/init_pose", "world_mocap", self.get_clock().now())


    # def transform_pose_old(self, pos, yaw):    
    #     """
    #     Given a vector3 position and yaw in mocap frame returns vector3 position and yaw in livox frame
        
    #     Input: Position in mocap frame, Vector3.
    #            Yaw in mocap frame, Float64.
    #     Output: Position in livox frame, Vector3.
    #             Yaw in livox frame, Float64. 
    #     """
    #     ps = PoseStamped()
    #     ps.header.stamp = self.get_clock().now().to_msg()
    #     ps.header.frame_id = "pos_goal"
    #     ps.pose.position.x = pos.x
    #     ps.pose.position.y = pos.y
    #     ps.pose.position.z = pos.z

    #     quat = quaternion_from_euler(0, 0, yaw)
    #     ps.pose.orientation.x = quat[0] 
    #     ps.pose.orientation.y = quat[1] 
    #     ps.pose.orientation.z = quat[2] 
    #     ps.pose.orientation.w = quat[3] 

    #     tf = self.get_transform()
    #     new_ps = do_transform_pose(ps.pose, tf)

    #     pos = Vector3()
    #     pos.x = new_ps.position.x
    #     pos.y = new_ps.position.y
    #     pos.z = new_ps.position.z
    #     yaw = euler_from_quat(new_ps.orientation)[-1]

    #     return pos, yaw  


    # def transform_vel_old(self, vel):    
    #     """
    #     Given a vector3 velocity in mocap frame output vector3 velocity message in livox frame

    #     Remember to zero out translation component of transform.

    #     NOTE: The angular velocity in the new frame depends on the derivatives of roll, pitch, and yaw in the original frame (omega_B = R_B_A @ omega_A). 
    #     Since we are only given dyaw, we would need to compute droll and dpitch from the equations of quadrotor dynamics. 
    #     But that is overkill since in this case dyaw is the same in both frames. 
        
    #     Input: Velocity in mocap frame, Vector3.
    #     Output: Velocity in livox frame, Vector3. 
    #     """

    #     tf= self.get_transform()

    #     # Constuct rot_only_tf from tf here
    #     rot_only_tf = TransformStamped()
    #     rot_only_tf.header = tf.header 
    #     rot_only_tf.child_frame_id = tf.child_frame_id
    #     rot_only_tf.transform.translation.x = 0.0
    #     rot_only_tf.transform.translation.y = 0.0
    #     rot_only_tf.transform.translation.z = 0.0
    #     rot_only_tf.transform.rotation = tf.transform.rotation 

    #     # Construct vector3stamped from vector3 
    #     v3s = Vector3Stamped()
    #     v3s.header.stamp = self.get_clock().now().to_msg()
    #     v3s.header.frame_id = "vel_goal"
    #     v3s.vector = vel 

    #     return do_transform_vector3(v3s, rot_only_tf).vector

    # def transform_acc_old(self, acc):    
    #     """
    #     Given a vector3 acceleration in mocap frame output vector3 acceleration in livox frame 

    #     Remember to zero out translational component of transform
    #     input: acceleration in mocap frame, Vector3.
    #     Output: acceleration in livox frame, Vector3.
    #     """

    #     tf= self.get_transform()

    #     # Constuct rot_only_tf from tf here
    #     rot_only_tf = TransformStamped()
    #     rot_only_tf.header = tf.header 
    #     rot_only_tf.child_frame_id = tf.child_frame_id
    #     rot_only_tf.transform.translation.x = 0.0
    #     rot_only_tf.transform.translation.y = 0.0
    #     rot_only_tf.transform.translation.z = 0.0
    #     rot_only_tf.transform.rotation = tf.transform.rotation

    #     # Construct vector3stamped from vector3 
    #     v3s = Vector3Stamped()
    #     v3s.header.stamp = self.get_clock().now().to_msg()
    #     v3s.header.frame_id = "acc_goal"
    #     v3s.vector = acc 

    #     return do_transform_vector3(v3s, rot_only_tf).vector

    # def goal_cb_old(self, msg):
    #     tf = self.get_transform() 
    #     new_pose = do_transform_pose(msg.pose, tf)

    #     new_ps = PoseStamped() 
    #     new_ps.header.stamp = msg.header.stamp
    #     new_ps.header.frame_id = msg.header.frame_id 
    #     new_ps.pose = new_pose 

    #     self.goal_publisher.publish(new_ps) 
        

    # def transform_cb_old(self, msg):
    #     """
    #     Given a dynus p, v, a, j, y, dyaw command in mocap frame,
    #     return a dynus p, v, a, j, y, dyaw command in livox frame.
        
    #     p, v, a, j are all vector3.
    #     yaw and dyaw are floats. 

    #     NOTE: 1. We don't transform jerk because px4 does not take jerk commands
    #           2. We don't transofrm dyaw because it is the same in both frames
    #     """

    #     p, v, a, j, yaw, dyaw = msg.p, msg.v, msg.a, msg.j, msg.yaw, msg.dyaw 
 
    #     # 1. Call a function that given a vector3 position and yaw returns vector3 position and yaw but in new frame
    #     new_p, new_yaw = self.transform_pose_old(p, yaw)
    #     # 2. Call a function that given a vector3 velocity returns vector3 velocity but in new frame
    #     new_v = self.transform_vel_old(v)
    #     # 3. Call a function that given a vector3 acceleration returns a vecotr3 acceleration in new frame. 
    #     new_a = self.transform_acc_old(a)
    #     # 4. Construct dynus Goal message and return

    #     livox_goal = Goal()
    #     livox_goal.header.stamp = self.get_clock().now().to_msg()
    #     livox_goal.header.frame_id = "traj_goal"
    #     livox_goal.p = new_p 
    #     livox_goal.v = new_v 
    #     livox_goal.a = new_a 
    #     livox_goal.j = j 
    #     livox_goal.yaw = new_yaw 
    #     livox_goal.dyaw = dyaw 

    #     self.dynus_publisher.publish(livox_goal)   

    # def pose_cb(self, msg):
    #     self.mocap_pose = msg 
    
    # def twist_cb(self, msg):
    #     self.mocap_twist = msg 

    # def broadcast_livox_frame(self):
    #     """
    #     Given pose and twist in mocap frame ,
    #     return pose and twist in livox frame.
    #     """
    #     if self.mocap_pose and self.mocap_twist:
    #         p = self.mocap_pose.pose.position 
    #         v = self.mocap_twist.twist.linear 
    #         yaw = euler_from_quat(self.mocap_pose.pose.orientation)[-1]
    #         dyaw = self.mocap_twist.twist.angular.z

    #         #TODO Implement 
    #         # 1. Call a function that given a vector3 position and yaw returns vector3 position and yaw but in new frame
    #         new_p, new_yaw = self.transform_pose(p, yaw)
    #         # 2. Call a function that given a vector3 velocity returns vector3 velocity but in new frame
    #         new_v = self.transform_vel(v)
    #         # 3. Call a function that given a vector3 acceleration returns a vecotr3 acceleration in new frame. 
            
    #         # Populate and broadcast transform
    #         tfs = TransformStamped()
    #         tfs.header.stamp = self.get_clock().now().to_msg()
    #         tfs.header.frame_id = f"{veh}/init_pose"
    #         tfs.child_frame_id = f"{veh}/livox_estimate"
    #         tfs.transform.translation = new_p 
    #         quat = quaternion_from_euler(0, 0, new_yaw)
    #         tfs.transform.rotation.x = quat[0] 
    #         tfs.transform.rotation.y = quat[1] 
    #         tfs.transform.rotation.z = quat[2] 
    #         tfs.transform.rotation.w = quat[3]

    #         broadcaster.sendTransform(tfs) 

    #         # Populate and publish velocity message
    #         lv_vel = TwistStamped()
    #         lv_vel.header.stamp = self.get_clock().now().to_msg()
    #         lv_vel.header.frame_id = "livox_vel"
    #         lv_vel.twist.linear = new_v
    #         lv_vel.twist.angular.z = dyaw

    #         self.vel_publisher.publish(lv_vel)

if __name__ == '__main__':
    main()