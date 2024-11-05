#/bin/usr/env python3 

import rclpy
from rclpy.serialization import deserialize_message
from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
from rosidl_runtime_py.utilities import get_message
import pandas as pd

from mavros_msgs.msg import AttitudeTarget
from geometry_msgs.msg import PoseStamped

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
            msg = deserialize_message(data, msg_type)

            roll, pitch, yaw = get_euler(msg)

            data_list.append({
                'timestamp': timestamp,
                'roll': roll,
                'pitch': pitch
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
    bag_path = '/home/juanrached/mavros_ws/bags/pid_response_1/rosbag2_2024_11_04-16_58_54'
    
    topic_name1 = '/mavros/setpoint_raw/target_attitude'
    output_csv1 = '/home/juanrached/mavros_ws/src/uwb_drone_experiments/data/pid_response/att_setpoints.csv'
    
    topic_name2 = '/mavros/local_position/pose'
    output_csv2 = '/home/juanrached/mavros_ws/src/uwb_drone_experiments/data/pid_response/att_measured.csv'

    read_bag_to_csv(bag_path, topic_name2, output_csv2)

if __name__ == '__main__':
    main()


