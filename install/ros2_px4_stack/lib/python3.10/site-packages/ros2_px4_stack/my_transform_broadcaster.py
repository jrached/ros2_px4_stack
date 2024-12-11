#!/usr/bin/env python

"""
import rospy, tf2_ros, geometry_msgs.msg
import geometry_msgs.eStamped.msg as PoseStamped

def callback(data):
    tf2Broadcast = tf2_ros.TransformBroadcaster()
    tf2Stamp = geometry_msgs.msg.TransformStamped()
    tf2Stamp.header.stamp = rospy.Time.now()
    tf2Stamp.header.frame_id = 'base_link'
    tf2Stamp.child_frame_id = 'base_laser'
    tf2Stamp.transform.translation = (0.1, 0.0, 0.2)
    tf2Stampl.transform.rotation = (0.0, 0.0, 0.0)
    tf2Broadcast.sendTransform(tf2Stamp)

if __name__ == '__main__':
    rospy.init_node('talker')
    rospy.Subscriber('/acl_alpha/world', PoseStamped, callback)
    rospy.spin()
"""

import rclpy
from rclpy.node import Node
import tf2_ros
from geometry_msgs.msg import PoseStamped

br = tf2_ros.TransformBroadcaster()

class FrameBroadcaster(Node):

    def __init__(self):
        super().__init__('agent_tf_broadcaster')

        self.agent_name = 'PX01'
        self.subscription = self.create_subscription(PoseStamped,
                                                    f'{self.agent_name}/world',
                                                    self.callback,
                                                    1)

    def callback(msg):
        #print('received')
        x = msg.pose.position.x
        y = msg.pose.position.y
        z = msg.pose.position.z
        rx = msg.pose.orientation.x
        ry = msg.pose.orientation.y
        rz = msg.pose.orientation.z
        rw = msg.pose.orientation.w
        #print(f'{x:.02f},{y:.02f},{z:.02f}')

        position = (x,y,z)
        orientation = (rx,ry,rz,rw)
        #orientation = tf2_ros.transformations.quaternion_from_euler(roll, pitch, yaw)
        t = self.get_clock().now()

        # from: mocap_world
        # to:   base_link
        # Ref: http://wiki.ros.org/tf/Tutorials/Writing%20a%20tf%20broadcaster%20%28Python%29
        br.sendTransform( position, orientation, t, 'base_link', 'world_mocap' ) 

if __name__ == '__main__':
    rclpy.init()
    node = FrameBroadcaster()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    
    rclpy.shutdown()
