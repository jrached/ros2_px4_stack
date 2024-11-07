#/bin/usr/env python3 

import rclpy
from rclpy.serialization import deserialize_message
from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
from rosidl_runtime_py.utilities import get_message
import pandas as pd

from mavros_msgs.msg import AttitudeTarget, PositionTarget
from geometry_msgs.msg import PoseStamped, TwistStamped
from std_msgs.msg import Header

from transforms3d.euler import quat2euler

def pos_bag_to_csv(bag_path, topic_name, output_csv):
    # Initialize ROS 2 and create a node
    rclpy.init()

    # Prepare the bag reader
    reader = SequentialReader()
    storage_options = {'uri': bag_path, 'storage_id': 'sqlite3'}
    converter_options = {'input_serialization_format': 'cdr', 'output_serialization_format': 'cdr'}
    storage_options = StorageOptions(uri=bag_path, storage_id='sqlite3')
    converter_options = ConverterOptions(
        input_serialization_format='cdr',
        output_serialization_format='cdr'
    )
    reader.open(storage_options, converter_options)
    
    # Get the type of the topic message
    topic_type = None
    for topic_metadata in reader.get_all_topics_and_types():
        if topic_metadata.name == topic_name:
            topic_type = topic_metadata.type
            break
    if not topic_type:
        print(f"Topic {topic_name} not found in the bag.")
        return
    
    # Initialize a list to collect data
    data_list = []
    
    # Deserialize messages and extract fields
    msg_type = get_message(topic_type)
    while reader.has_next():
        (topic, data, timestamp) = reader.read_next()
        
        if topic == topic_name:
            header = deserialize_message(data, Header)
            msg = deserialize_message(data, msg_type)
            nanosecs = header.stamp.nanosec

            # roll, pitch, yaw = get_euler(msg)

            data_list.append({
                'timestamp': timestamp,
                'nanoseconds': nanosecs,
                'x': msg.pose.position.x,
                'y': msg.pose.position.y,
                'z': msg.pose.position.z
            })

    # Convert to a DataFrame and write to CSV
    df = pd.DataFrame(data_list)
    df.to_csv(output_csv, index=False)

    print(f"Data from topic '{topic_name}' has been written to {output_csv}.")
    rclpy.shutdown()


def vel_bag_to_csv(bag_path, topic_name, output_csv):
    # Initialize ROS 2 and create a node
    rclpy.init()

    # Prepare the bag reader
    reader = SequentialReader()
    storage_options = {'uri': bag_path, 'storage_id': 'sqlite3'}
    converter_options = {'input_serialization_format': 'cdr', 'output_serialization_format': 'cdr'}
    storage_options = StorageOptions(uri=bag_path, storage_id='sqlite3')
    converter_options = ConverterOptions(
        input_serialization_format='cdr',
        output_serialization_format='cdr'
    )
    reader.open(storage_options, converter_options)
    
    # Get the type of the topic message
    topic_type = None
    for topic_metadata in reader.get_all_topics_and_types():
        if topic_metadata.name == topic_name:
            topic_type = topic_metadata.type
            break
    if not topic_type:
        print(f"Topic {topic_name} not found in the bag.")
        return
    
    # Initialize a list to collect data
    data_list = []
    
    # Deserialize messages and extract fields
    msg_type = get_message(topic_type)
    while reader.has_next():
        (topic, data, timestamp) = reader.read_next()
        
        if topic == topic_name:
            header = deserialize_message(data, Header)
            msg = deserialize_message(data, msg_type)
            nanosecs = header.stamp.nanosec

            # roll, pitch, yaw = get_euler(msg)
            if isinstance(msg, TwistStamped):
                data_list.append({
                    'timestamp': timestamp,
                    'nanoseconds': nanosecs,
                    'x': msg.twist.linear.x,
                    'y': msg.twist.linear.y,
                    'z': msg.twist.linear.z
                })
            elif isinstance(msg, PositionTarget):
                data_list.append({
                    'timestamp': timestamp,
                    'nanoseconds': nanosecs,
                    'x': msg.velocity.x,
                    'y': msg.velocity.y,
                    'z': msg.velocity.z
                })
            else:
                raise Exception(f"Did not expect topic of type {topic_type}")

    # Convert to a DataFrame and write to CSV
    df = pd.DataFrame(data_list)
    df.to_csv(output_csv, index=False)

    print(f"Data from topic '{topic_name}' has been written to {output_csv}.")
    rclpy.shutdown()


def att_bag_to_csv(bag_path, topic_name, output_csv):
    # Initialize ROS 2 and create a node
    rclpy.init()

    # Prepare the bag reader
    reader = SequentialReader()
    storage_options = {'uri': bag_path, 'storage_id': 'sqlite3'}
    converter_options = {'input_serialization_format': 'cdr', 'output_serialization_format': 'cdr'}
    storage_options = StorageOptions(uri=bag_path, storage_id='sqlite3')
    converter_options = ConverterOptions(
        input_serialization_format='cdr',
        output_serialization_format='cdr'
    )
    reader.open(storage_options, converter_options)
    
    # Get the type of the topic message
    topic_type = None
    for topic_metadata in reader.get_all_topics_and_types():
        if topic_metadata.name == topic_name:
            topic_type = topic_metadata.type
            break
    if not topic_type:
        print(f"Topic {topic_name} not found in the bag.")
        return
    
    # Initialize a list to collect data
    data_list = []
    
    # Deserialize messages and extract fields
    msg_type = get_message(topic_type)
    while reader.has_next():
        (topic, data, timestamp) = reader.read_next()
        
        if topic == topic_name:
            header = deserialize_message(data, Header)
            msg = deserialize_message(data, msg_type)
            nanosecs = header.stamp.nanosec

            roll, pitch, yaw = get_euler(msg)

            data_list.append({
                'timestamp': timestamp,
                'nanoseconds': nanosecs,
                'roll': roll,
                'pitch': pitch
            })

    # Convert to a DataFrame and write to CSV
    df = pd.DataFrame(data_list)
    df.to_csv(output_csv, index=False)

    print(f"Data from topic '{topic_name}' has been written to {output_csv}.")
    rclpy.shutdown()

def get_euler(msg):
    if isinstance(msg, PoseStamped):
        pose = msg.pose
    elif isinstance(msg, AttitudeTarget):
        pose = msg
    else:
        raise Exception(f"Did not expect topic of type {type(msg)}")

    qx = pose.orientation.x  
    qy = pose.orientation.y
    qz = pose.orientation.z
    qw = pose.orientation.w

    return quat2euler([qw, qx, qy, qz], axes='sxyz')

def main():
    # att_bag_path = '/home/juanrached/mavros_ws/bags/pid_response_1/rosbag2_2024_11_04-16_58_54'
    # pos_and_att_bag_path = '/home/juanrached/mavros_ws/bags/pid_response_7/rosbag2_2024_11_06-16_39_22'

    # New #############################################################################################################
    topic_name1 = '/mavros/setpoint_position/local'
    topic_name2 = '/mavros/local_position/pose'
    topic_name3 = '/mavros/setpoint_raw/target_attitude'
    topic_name4 = '/mavros/setpoint_raw/target_local'
    topic_name5 = '/PX01/mocap/twist'
    
    test_num = 8 
    bag_path = f"/home/juanrached/mavros_ws/bags/pid_response_{test_num}/" #TODO add bag filename
    output_path = f"/home/juanrached/mavros_ws/src/uwb_drone_experiments/data/pid_response_{test_num}"

    pos_bag_to_csv(bag_path, topic_name2, output_path + "pos_measured.csv") 
    pos_bag_to_csv(bag_path, topic_name1, output_path + "pos_setpoints.csv")
    
    vel_bag_to_csv(bag_path, topic_name5, output_path + "vel_measured.csv") 
    vel_bag_to_csv(bag_path, topic_name4, output_path + "vel_setpoints.csv") 

    att_bag_to_csv(bag_path, topic_name2, output_path + "att_measured.csv")  
    att_bag_to_csv(bag_path, topic_name3, output_path + "att_setpoints.csv") 

if __name__ == '__main__':
    main()


