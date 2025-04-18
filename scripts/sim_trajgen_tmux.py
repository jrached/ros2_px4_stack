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

        # Split the terminal into a 2x2 grid
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
            full_command = f"cd ~/code/mavros_ws && source install/setup.bash && {cmd}"
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
        f"ros2 launch mavros px4.launch fcu_url:='udp://:14540@localhost:14557' namespace:={veh}/mavros tgt_system:={mav_id}",  # Command for pane 1
        "ros2 launch trajectory_generator_ros2 onboard.launch.py",  # Command for pane 2
        "ros2 launch trajectory_generator_ros2 base_station.launch.py",  # Command for pane 3
        "ros2 launch ros2_px4_stack livox_gen_traj.launch.py",  # Command for pane 4,
        f"cd ~/acl/px4/PX4-Autopilot && make px4_sitl gz_x500", # Pane 5
        f"sleep 10.0 && ros2 topic echo {veh}/mavros/local_position/pose", # Pane 6
        f"sleep 10.0 && ros2 topic echo {veh}/goal", # Pane  7
        f"sleep 10.0 && cd bags && cd sim_tests && rm -rf sim_bag* && ros2 bag record -o sim_bag /SQ01/goal {veh}/mavros/local_position/pose {veh}/mavros/local_position/velocity_local", # Pane 9
        "echo ros2 service call /change_mode mission_mode/srv/MissionModeChange \\\"{mode: 1}\\\" | xclip -selection clipboard", # Pane 8
    ]
    run_tmux_commands(session_name, commands)