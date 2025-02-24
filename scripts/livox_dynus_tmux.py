#!/bin/usr/env python3

import subprocess
import os 

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
    commands = [
        f"source ~/code/dynus_ws/install/setup.bash && source ~/code/decomp_ws/install/setup.bash && sleep 10 && ros2 launch dynus onboard_dynus.launch.py x:=0.0 y:=0.0 z:=0.0 yaw:=0 namespace:={veh} use_obstacle_tracker:=false use_ground_robot:=false use_hardware:=true " \
        "use_onboard_localization:=true depth_camera_name:=d455",  # Command for pane 1

        "echo 'not using d455'", # Pane 2

        f"source ~/code/livox_ws/install/setup.bash && sleep 10 && ros2 launch livox_ros_driver2 run_MID360_launch.py namespace:={veh}", # Pane 3

        f"source ~/code/dynus_ws/install/setup.bash && source ~/code/dlio_ws/install/setup.bash && sleep 10 && ros2 launch direct_lidar_inertial_odometry dlio.launch.py namespace:={veh}", # Pane 4

        f"ros2 launch mavros px4.launch namespace:={veh}/mavros tgt_system:={mav_id}", # Pane 5
        #"echo mavros",

        f"sleep 15.0 && ros2 topic echo {veh}/mavros/local_position/pose", # Pane 6

        f"sleep 15.0 && ros2 topic echo /{veh}/dlio/odom_node/pose", # Pane  7

        f"source ~/code/dynus_ws/install/setup.bash && source ~/code/get_init_pose.sh && sleep 10 && ros2 launch ros2_px4_stack dynus_mavros.launch.py", # Pane 8

        # f"sleep 30.0 && source ~/code/dynus_ws/install/setup.bash && cd ~/code/data/bags && rm -rf rosbag* && ros2 bag record {veh}/mavros/local_position/odom {veh}/mavros/setpoint_trajectory/local {veh}/dlio/odom_node/odom {veh}/goal", # Pane 9
        # f"sleep 30.0 && source ~/code/dynus_ws/install/setup.bash && cd ~/code/data/bags && rm -rf rosbag* && ros2 bag record PX02/mavros/local_position/pose PX03/mavros/local_position/pose /trajs {veh}/dynamic_map_marker /tf /tf_static {veh}/goal {veh}/traj_committed_colored", 
        #f"sleep 30.0 && source ~/code/dynus_ws/install/setup.bash && cd ~/code/data/bags && rm -rf rosbag* && ros2 bag record PX01/dlio/odom_node/pose PX02/dlio/odom_node/pose PX03/dlio/odom_node/pose", # Pane 9
        f"sleep 30.0 && source ~/code/dynus_ws/install/setup.bash && cd ~/code/data/bags && rm -rf rosbag* && ros2 bag record {veh}/mavros/local_position/pose /trajs {veh}/dynamic_map_marker /tf /tf_static {veh}/goal {veh}/traj_committed_colored", 

        
    ]
    run_tmux_commands(session_name, commands)




