#!/usr/bin/env python3

import os 
import rclpy 
from rclpy.node import Node 
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from geometry_msgs.msg import PoseStamped, TwistStamped, TransformStamped, Vector3, Vector3Stamped  
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

        # Make dynus goal topic subscription 
        self.dynus_subscription = self.create_subscription(Goal, f"/{veh}/goal", self.transform_cb, 10)

        # Make dynus goal topic publisher 
        self.dynus_publisher = self.create_publisher(Goal, f"/{veh}/agent_frame_goal", 10)

        # Make transform listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)


        ### For Debugging ### 
        # # Make mocap subscription
        # self.pose_subscription = self.create_subscription(PoseStamped, f"/{veh}/world", self.pose_cb, 10)
        # self.twist_subscription = self.create_subscription(TwistStamped, f"/{veh}/mocap/twist", self.twist_cb, 10) 
        
        # Define velocity publisher 
        # self.vel_publisher = self.create_publisher(TwistStamped, f"/{veh}/livox_vel_estimate", 10)

        # Make timer for transform broadcaster
        # pub_freq = 50 # Hz 
        # timer_period = 1 / pub_freq 
        # self.timer = self.create_timer(timer_period, self.broadcast_livox_frame)

        # Class variables to store mocap pose and twist
        self.mocap_pose = None
        self.mocap_twist = None 


    def get_transform(self):
        # Get static transform betweenf livox initial position and mocap origin
        while not self.tf_buffer.can_transform(f"{veh}/init_pose", "world_mocap", self.get_clock().now()):
            self.get_logger().info(f"Waiting for transform")
            rclpy.spin_once(self, timeout_sec=1.0)

        return self.tf_buffer.lookup_transform(f"{veh}/init_pose", "world_mocap", self.get_clock().now())


    def transform_pose(self, pos, yaw):    
        """
        Given a vector3 position and yaw in mocap frame returns vector3 position and yaw in livox frame
        
        Input: Position in mocap frame, Vector3.
               Yaw in mocap frame, Float64.
        Output: Position in livox frame, Vector3.
                Yaw in livox frame, Float64. 
        """
        ps = PoseStamped()
        ps.header.stamp = self.get_clock().now().to_msg()
        ps.header.frame_id = "pos_goal"
        ps.pose.position.x = pos.x
        ps.pose.position.y = pos.y
        ps.pose.position.z = pos.z

        quat = quaternion_from_euler(0, 0, yaw)
        ps.pose.orientation.x = quat[0] 
        ps.pose.orientation.y = quat[1] 
        ps.pose.orientation.z = quat[2] 
        ps.pose.orientation.w = quat[3] 

        tf = self.get_transform()
        new_ps = do_transform_pose(ps.pose, tf)

        pos = Vector3()
        pos.x = new_ps.position.x
        pos.y = new_ps.position.y
        pos.z = new_ps.position.z
        yaw = euler_from_quat(new_ps.orientation)[-1]

        return pos, yaw  


    def transform_vel(self, vel):    
        """
        Given a vector3 velocity in mocap frame output vector3 velocity message in livox frame

        Remember to zero out translation component of transform.

        NOTE: The angular velocity in the new frame depends on the derivatives of roll, pitch, and yaw in the original frame (omega_B = R_B_A @ omega_A). 
        Since we are only given dyaw, we would need to compute droll and dpitch from the equations of quadrotor dynamics. 
        But that is overkill since in this case dyaw is the same in both frames. 
        
        Input: Velocity in mocap frame, Vector3.
        Output: Velocity in livox frame, Vector3. 
        """

        tf= self.get_transform()

        # Constuct rot_only_tf from tf here
        rot_only_tf = TransformStamped()
        rot_only_tf.header = tf.header 
        rot_only_tf.child_frame_id = tf.child_frame_id
        rot_only_tf.transform.translation.x = 0.0
        rot_only_tf.transform.translation.y = 0.0
        rot_only_tf.transform.translation.z = 0.0
        rot_only_tf.transform.rotation = tf.transform.rotation 

        # Construct vector3stamped from vector3 
        v3s = Vector3Stamped()
        v3s.header.stamp = self.get_clock().now().to_msg()
        v3s.header.frame_id = "vel_goal"
        v3s.vector = vel 

        return do_transform_vector3(v3s, rot_only_tf).vector


    def transform_acc(self, acc):    
        """
        Given a vector3 acceleration in mocap frame output vector3 acceleration in livox frame 

        Remember to zero out translational component of transform
        input: acceleration in mocap frame, Vector3.
        Output: acceleration in livox frame, Vector3.
        """

        tf= self.get_transform()

        # Constuct rot_only_tf from tf here
        rot_only_tf = TransformStamped()
        rot_only_tf.header = tf.header 
        rot_only_tf.child_frame_id = tf.child_frame_id
        rot_only_tf.transform.translation.x = 0.0
        rot_only_tf.transform.translation.y = 0.0
        rot_only_tf.transform.translation.z = 0.0
        rot_only_tf.transform.rotation = tf.transform.rotation

        # Construct vector3stamped from vector3 
        v3s = Vector3Stamped()
        v3s.header.stamp = self.get_clock().now().to_msg()
        v3s.header.frame_id = "acc_goal"
        v3s.vector = acc 

        return do_transform_vector3(v3s, rot_only_transform).vector

    def transform_cb(self, msg):
        """
        Given a dynus p, v, a, j, y, dyaw command in mocap frame,
        return a dynus p, v, a, j, y, dyaw command in livox frame.
        
        p, v, a, j are all vector3.
        yaw and dyaw are floats. 

        NOTE: 1. We don't transform jerk because px4 does not take jerk commands
              2. We don't transofrm dyaw because it is the same in both frames
        """

        p, v, a, j, yaw, dyaw = msg.p, msg.v, msg.a, msg.j, msg.yaw, msg.dyaw 
 
        # 1. Call a function that given a vector3 position and yaw returns vector3 position and yaw but in new frame
        new_p, new_yaw = self.transform_pose(p, yaw)
        # 2. Call a function that given a vector3 velocity returns vector3 velocity but in new frame
        new_v = self.transform_vel(v)
        # 3. Call a function that given a vector3 acceleration returns a vecotr3 acceleration in new frame. 
        new_acc = self.transform_acc(a)
        # 4. Construct dynus Goal message and return

        livox_goal = Goal()
        livox_goal.header.stamp = self.get_clock().now().to_msg()
        livox_goal.header.frame_id = "traj_goal"
        livox_goal.p = new_p 
        livox_goal.v = new_v 
        livox_goal.a = new_a 
        livox_goal.j = j 
        livox_goal.yaw = new_yaw 
        livox_goal.dyaw = dyaw 

        self.dynus_publisher.publish(livox_goal)   

    def pose_cb(self, msg):
        self.mocap_pose = msg 
    
    def twist_cb(self, msg):
        self.mocap_twist = msg 

    def broadcast_livox_frame(self):
        """
        Given pose and twist in mocap frame ,
        return pose and twist in livox frame.
        """
        if self.mocap_pose and self.mocap_twist:
            p = self.mocap_pose.pose.position 
            v = self.mocap_twist.twist.linear 
            yaw = euler_from_quat(self.mocap_pose.pose.orientation)[-1]
            dyaw = self.mocap_twist.twist.angular.z

            #TODO Implement 
            # 1. Call a function that given a vector3 position and yaw returns vector3 position and yaw but in new frame
            new_p, new_yaw = self.transform_pose(p, yaw)
            # 2. Call a function that given a vector3 velocity returns vector3 velocity but in new frame
            new_v = self.transform_vel(v)
            # 3. Call a function that given a vector3 acceleration returns a vecotr3 acceleration in new frame. 
            
            # Populate and broadcast transform
            tfs = TransformStamped()
            tfs.header.stamp = self.get_clock().now().to_msg()
            tfs.header.frame_id = f"{veh}/init_pose"
            tfs.child_frame_id = f"{veh}/livox_estimate"
            tfs.transform.translation = new_p 
            quat = quaternion_from_euler(0, 0, new_yaw)
            tfs.transform.rotation.x = quat[0] 
            tfs.transform.rotation.y = quat[1] 
            tfs.transform.rotation.z = quat[2] 
            tfs.transform.rotation.w = quat[3]

            broadcaster.sendTransform(tfs) 

            # Populate and publish velocity message
            lv_vel = TwistStamped()
            lv_vel.header.stamp = self.get_clock().now().to_msg()
            lv_vel.header.frame_id = "livox_vel"
            lv_vel.twist.linear = new_v
            lv_vel.twist.angular.z = dyaw

            self.vel_publisher.publish(lv_vel)


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

if __name__ == '__main__':
    main()