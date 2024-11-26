#!/bin/usr/env python3

import subprocess

def run_tmux_commands(session_name, commands):
    """
    Set up a TMUX session with multiple windows and run specific commands in each.
    
    Args:
        session_name (str): Name of the TMUX session.
        commands (list): A list of commands to execute in each window.
    """
    try:
        # Start a new TMUX session
        subprocess.run(["tmux", "new-session", "-d", "-s", session_name], check=True)
        
        for i, cmd in enumerate(commands):
            if i == 0:
                # For the first window (created automatically), just rename it
                subprocess.run(["tmux", "rename-window", "-t", f"{session_name}:0", f"window_{i+1}"], check=True)
            else:
                # Create new windows for the subsequent commands
                subprocess.run(["tmux", "new-window", "-t", session_name, "-n", f"window_{i+1}"], check=True)
            
            # Send the commands to the window
            command_sequence = f"cd ~/mavros_ws && source install/setup.bash && {cmd}"
            subprocess.run(["tmux", "send-keys", "-t", f"{session_name}:window_{i+1}", command_sequence, "C-m"], check=True)
        
        # Attach to the session
        print(f"TMUX session '{session_name}' created. Attaching...")
        subprocess.run(["tmux", "attach-session", "-t", session_name])
    
    except subprocess.CalledProcessError as e:
        print(f"Error setting up TMUX session: {e}")

if __name__ == "__main__":
    # Define the session name and commands
    session_name = "px01_tmux_session"
    commands = [
        "ros2 launch mavros px4.launch",  # Command for window 1
        "ros2 launch trajectory_generator_ros2 onboard.launch.py",  # Command for window 2
        "ros2 launch trajectory_generator_ros2 base_station.launch.py",  # Command for window 3
        "ros2 launch ros2_px4_stack offboard_gen_traj.launch.py",  # Command for window 4
    ]
    run_tmux_commands(session_name, commands)
