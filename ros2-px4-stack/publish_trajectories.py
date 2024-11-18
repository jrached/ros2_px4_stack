#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, PoseStamped, Point, Quaternion
from std_msgs.msg import Header
import pandas as pd
import numpy as np

from drone_vars import ROS_NAMESPACES, TRAJ_FPATH


def get_trajectories_from_df(df):
    trajs = {}
    for ns in ROS_NAMESPACES:
        x = df[f"{ns}_x"].values
        y = df[f"{ns}_y"].values
        z = df[f"{ns}_z"].values
        qx = df[f"{ns}_qx"].values
        qy = df[f"{ns}_qy"].values
        qz = df[f"{ns}_qz"].values
        qw = df[f"{ns}_qw"].values
        traj = [
            Pose(
                position=Point(x=x[i], y=y[i], z=z[i]),
                orientation=Quaternion(x=qx[i], y=qy[i], z=qz[i], w=qw[i]),
            )
            for i in range(len(x))
        ]
        trajs[ns] = traj

    # make sure that all of the trajectories are the same length
    traj_lengths = [len(trajs[ns]) for ns in ROS_NAMESPACES]
    assert all([length == traj_lengths[0] for length in traj_lengths])

    return trajs


if __name__ == "__main__":

    # make a ros node
    rclpy.init()
    node = rclpy.create_node("publish_trajectories")

    # read the trajectory dataframe and check the timing
    traj_df = pd.read_csv(TRAJ_FPATH)
    times = traj_df["time"].values
    print(f"Times: {times}")
    traj_update_interval = times[1] - times[0]
    time_diffs = np.diff(times)
    assert np.allclose(time_diffs, traj_update_interval), f"Time diffs: {time_diffs}"

    #  get the trajectories from the dataframe
    trajs = get_trajectories_from_df(traj_df)

    # print the z values for each namespace
    for ns in ROS_NAMESPACES:
        z_values = [pose.position.z for pose in trajs[ns]]
        print(f"{ns} z values: {z_values}")

    # make publishers for each namespace that should be: /<namespace>/offboard/setpoint of type PoseStamped
    traj_setpoint_pubs = {
        ns: node.create_publisher(PoseStamped, f"/{ns}/offboard/setpoint", 10)
        for ns in ROS_NAMESPACES
    }

    # loop over the times and publish the setpoints
    rate = node.create_rate(1 / traj_update_interval)
    curr_traj_idx = 0
    while rclpy.ok():
        for ns, traj in trajs.items():
            pose = traj[curr_traj_idx]
            setpoint = PoseStamped(
                header=Header(stamp=node.get_clock().now(), frame_id="map"), pose=pose
            )
            traj_setpoint_pubs[ns].publish(setpoint)

        rate.sleep()
        curr_traj_idx = (curr_traj_idx + 1) % len(times)
