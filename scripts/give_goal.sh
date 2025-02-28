#!/bin/bash 

pos_x=$1
pos_y=$2
pos_z=$3

ros2 topic pub /PX01/term_goal geometry_msgs/msg/PoseStamped "{header: {stamp: {sec: 0.0, nanosec: 0.0}, frame_id: map}, pose: {position: {x: ${pos_x}, y: ${pos_y}, z: ${pos_z}}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}" -1 