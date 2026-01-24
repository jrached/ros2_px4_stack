#!/bin/usr/env python3

import subprocess
import os 
import argparse 

def run_tmux_commands(session_name, commands):
    """
    Set up a TMUX session with a 2x2 grid of panes, each running a specific command.

    Args:
        session_name (str): Name of the TMUX session.
        commands (list): A list of commands to execute in each pane (up to 4).
    """
    try:
        # Start a new TMUX session
        subprocess.run(["tmux", "new-session", "-d", "-s", session_name], check=True)

        # Split the terminal into a 3x3 grid
        subprocess.run(["tmux", "split-window", "-h", "-t", f"{session_name}:0"], check=True)  # Split horizontally
        subprocess.run(["tmux", "split-window", "-v", "-t", f"{session_name}:0.0"], check=True)  # Split top vertically
        subprocess.run(["tmux", "split-window", "-v", "-t", f"{session_name}:0.1"], check=True)  # Split bottom vertically
        subprocess.run(["tmux", "split-window", "-h", "-t", f"{session_name}:0.2"], check=True) # Split bottom-left horizontally
        subprocess.run(["tmux", "split-window", "-h", "-t", f"{session_name}:0.3"], check=True) # Split bottom-right horizontally
        subprocess.run(["tmux", "split-window", "-h", "-t", f"{session_name}:0.4"], check=True) # Split bottom-left horizontally
        subprocess.run(["tmux", "split-window", "-v", "-t", f"{session_name}:0.5"], check=True) # Split bottom-right vertically
        subprocess.run(["tmux", "split-window", "-v", "-t", f"{session_name}:0.0"], check=True) # Split bottom-right vertically
        subprocess.run(["tmux", "split-window", "-h", "-t", f"{session_name}:0.0"], check=True) # Split last horizontally
        subprocess.run(["tmux", "split-window", "-h", "-t", f"{session_name}:0.0"], check=True) # Split last horizontally

        # Commands to run in each pane
        for i, cmd in enumerate(commands):
            # Construct the full command with setup steps
            full_command = f"source ~/code/mavros_ws/install/setup.bash && source ~/code/bridge_ws/install/setup.bash && {cmd}"
            # Send the command to the corresponding pane
            subprocess.run(["tmux", "send-keys", "-t", f"{session_name}:0.{i}", full_command, "C-m"], check=True)

        # Select the first pane and arrange in an even grid
        subprocess.run(["tmux", "select-layout", "-t", session_name, "tiled"], check=True)

        # Attach to the session
        print(f"TMUX session '{session_name}' created with a 3x3 grid. Attaching...")
        subprocess.run(["tmux", "attach-session", "-t", session_name])

    except subprocess.CalledProcessError as e:
        print(f"Error setting up TMUX session: {e}")

if __name__ == "__main__":
    # Define the session name and commands
    veh = os.environ.get("VEH_NAME")
    mav_id = os.environ.get("MAV_SYS_ID")
    session_name = f"{veh}_tmux_session"

    # Get odom_type parameters from user 
    parser = argparse.ArgumentParser() 
    parser.add_argument('--odom_type', type=str, default="livox", help="Odometry measurement to send flight controller - motion capture or lidar")
    parser.add_argument('--planner', type=str, default="dynus", help="Planner - dynus or mighty")
    args = parser.parse_args()
    planner = args.planner
    odom_type = args.odom_type 

    commands = [
        f"source ~/code/{planner}_ws/install/setup.bash && source ~/code/decomp_ws/install/setup.bash && ros2 launch {planner} onboard_{planner}.launch.py x:=0.0 y:=0.0 z:=0.0 yaw:=0 namespace:={veh} use_obstacle_tracker:=false use_ground_robot:=false use_hardware:=true " \
        "use_onboard_localization:=true depth_camera_name:=d455",  # Command for pane 1

        # f"source ~/code/realsense-ros_ws/install/setup.bash && sleep 10 && ros2 launch realsense2_camera rs_d455_launch.py camera_namespace:=PX01 align_depth:=true depth_module.profile:=848x480x15 color_module.profile:=640x480x15", # Pane  2

        f"source ~/code/livox_ws/install/setup.bash && sleep 10 && ros2 launch livox_ros_driver2 run_MID360_launch.py namespace:={veh}", # Pane 3

        f"source ~/code/dynus_ws/install/setup.bash && source ~/code/dlio_ws/install/setup.bash && sleep 10 && ros2 launch direct_lidar_inertial_odometry dlio.launch.py namespace:={veh}", # Pane 4

        f"sleep 5.0 && ros2 launch mavros px4.launch namespace:={veh}/mavros tgt_system:={mav_id}", # Pane 5

        f"source ~/code/dynus_ws/install/setup.bash && sleep 10 && source ~/code/mavros_ws/src/ros2_px4_stack/scripts/trajectories/get_init_pose.sh && ros2 launch ros2_px4_stack dynus_mavros.launch.py odom_type:={odom_type}", # Pane 6
        # f"sleep 30.0 && source ~/code/dynus_ws/install/setup.bash && cd ~/code/data/bags && rm -rf rosbag*", # Pane 7
        # f"sleep 10.0 && source ~/code/dynus_ws/install/setup.bash && cd ~/data && rm -rf * && ros2 bag record -a -o twist_bag", # Pane 7 # /tf /tf_static

        # f"sleep 15 && source /home/swarm/code/decomp_ws/install/setup.bash && rm -rf /home/swarm/data/num_1 && source /home/swarm/code/dynus_ws/install/setup.bash && python3 /home/swarm/code/dynus_ws/src/dynus/scripts/bag_record.py --bag_number 1 --bag_path /home/swarm/data",
        f"sleep 15.0 && source ~/code/decomp_ws/install/setup.bash && source ~/code/dynus_ws/install/setup.bash && cd ~/data && ros2 bag record /trajs {veh}/world {veh}/mavros/local_position/pose {veh}/dlio/odom_node/pose {veh}/mavros/setpoint_trajectory/local {veh}/mavros/imu/data_raw {veh}/mavros/imu/data /tf /tf_static", # Pane 7
        
        f"sleep 15.0 && ros2 topic echo {veh}/mavros/local_position/pose", # Pane 8 

        'sleep 10.0 && source ~/code/mavros_ws/src/ros2_px4_stack/scripts/trajectories/get_init_pose.sh && echo && echo "init pos: (${INIT_X}, ${INIT_Y}, ${INIT_Z})" && echo "init att: (${INIT_ROLL}, ${INIT_PITCH}, ${INIT_YAW})" && echo', # Pane 9

        f"zenoh_router", # Pane 10
        f"ros2 launch global_mapper_ros global_mapper_node.launch.py quad:={veh} depth_pointcloud_topic:=livox/lidar",
    ]
    run_tmux_commands(session_name, commands)






