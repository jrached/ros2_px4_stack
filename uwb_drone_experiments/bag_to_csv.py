#/bin/usr/env python3 

import rclpy
from rclpy.serialization import deserialize_message
from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
from rosidl_runtime_py.utilities import get_message
import pandas as pd

from mavros_msgs.msg import AttitudeTarget
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Header

from transforms3d.euler import quat2euler

def read_bag_to_csv(bag_path, topic_name, output_csv):
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

def get_euler(msg):
    qx = msg.pose.orientation.x  
    qy = msg.pose.orientation.y
    qz = msg.pose.orientation.z
    qw = msg.pose.orientation.w

    return quat2euler([qw, qx, qy, qz], axes='sxyz')

def main():
    # Use the function
    bag_path = '/home/juanrached/mavros_ws/bags/pid_response_2/rosbag2_2024_11_05-16_54_20'
    
    topic_name1 = '/mavros/setpoint_position/local'
    output_csv1 = '/home/juanrached/mavros_ws/src/uwb_drone_experiments/data/pid_response2/pos_setpoints.csv'
    
    topic_name2 = '/mavros/local_position/pose'
    output_csv2 = '/home/juanrached/mavros_ws/src/uwb_drone_experiments/data/pid_response2/pos_measured.csv'

    read_bag_to_csv(bag_path, topic_name1, output_csv1)

if __name__ == '__main__':
    main()


