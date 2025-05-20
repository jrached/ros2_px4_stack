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

        # Split the terminal into a 2x2 grid
        subprocess.run(["tmux", "split-window", "-h", "-t", f"{session_name}:0"], check=True)  # Split horizontally
        subprocess.run(["tmux", "split-window", "-v", "-t", f"{session_name}:0.0"], check=True)  # Split top vertically
        subprocess.run(["tmux", "split-window", "-v", "-t", f"{session_name}:0.1"], check=True)  # Split bottom vertically
        subprocess.run(["tmux", "split-window", "-h", "-t", f"{session_name}:0.2"], check=True) # Split bottom-left horizontally
        subprocess.run(["tmux", "split-window", "-h", "-t", f"{session_name}:0.3"], check=True) # Split bottom-right horizontally
        subprocess.run(["tmux", "split-window", "-h", "-t", f"{session_name}:0.4"], check=True) # Split bottom-left horizontally
        subprocess.run(["tmux", "split-window", "-v", "-t", f"{session_name}:0.5"], check=True) # Split bottom-right vertically
        subprocess.run(["tmux", "split-window", "-h", "-t", f"{session_name}:0.0"], check=True) # Split last horizontally
        subprocess.run(["tmux", "split-window", "-v", "-t", f"{session_name}:0.0"], check=True) # Split bottom-right vertically

        # Commands to run in each pane
        for i, cmd in enumerate(commands):
            # Construct the full command with setup steps
            full_command = f"source ~/code/trajgen_ws/install/setup.bash && source ~/code/bridge_ws/install/setup.bash && source ~/code/mavros_ws/install/setup.bash && {cmd}"
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
    parser.add_argument('--odom_type', type=str, default="mocap", help="Odometry measurement to send flight controller - motion capture or lidar")
    args = parser.parse_args()
    odom_type = args.odom_type 

    if odom_type == "mocap":
        gt_odom_topic = "/world"
    else: 
        gt_odom_topic = "/dlio/odom_node/pose"
        
    commands = [
        f"ros2 launch mavros px4.launch namespace:={veh}/mavros tgt_system:={mav_id}",  # Command for pane 1
        "ros2 launch trajectory_generator_ros2 onboard.launch.py",  # Command for pane 2
        "ros2 launch trajectory_generator_ros2 base_station.launch.py",  # Command for pane 3
        f"ros2 launch ros2_px4_stack offboard_gen_traj.launch.py odom_type:={odom_type}",  # Command for pane 4,
        f"source ~/code/livox_ws/install/setup.bash && sleep 10 && ros2 launch livox_ros_driver2 run_MID360_launch.py namespace:={veh}", # Pane 5
        f"source ~/code/dynus_ws/install/setup.bash && source ~/code/dlio_ws/install/setup.bash && sleep 10 && ros2 launch direct_lidar_inertial_odometry dlio.launch.py namespace:={veh}", # Pane 6
        f"sleep 10.0 && ros2 topic echo {veh}/mavros/local_position/pose", # Pane 7
        f"sleep 10.0 && ros2 topic echo /{veh}" + gt_odom_topic, # Pane  8
        "sleep 10.0 && ros2 topic echo /SQ01/goal", 
        "zenoh_router", # Pane 10
    ]
    run_tmux_commands(session_name, commands)

