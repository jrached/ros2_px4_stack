# ros2-px4-stack
Migration of uwb_drone_experiments to ros2

# Pixhawk Setup

1. On your pc install PX-Autopilot:
    https://docs.px4.io/main/en/dev_setup/building_px4.html 
    
   You should just have to run: 

   git clone https://github.com/PX4/PX4-Autopilot.git --recursive

   The rest of the guide is for reference.

2. Then connect the pixhawk to your pc via usb and follow this guide to download QGroundControl, load the PX4-Autopilot firmware into the pixhawk, and configure the flight controller.

    https://docs.px4.io/main/en/config/ 


# Companion Computer Setup

1. On the companion computer, install ROS2 Humble:
    https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html 

2. Install Mavros following the installation section of this readme: 
    https://github.com/mavlink/mavros/blob/ros2/mavros/README.md
    
    Make sure to get the release for the humble distribution.

3. Create a workspace:
    mkdir -p mavros_ws/src

4. Inside the mavros_ws/src directory, git clone this repo:

    git clone https://github.com/jrached/uwb_drone_experiments.git


# Running the code

Note: This code assumes some form of position estimate being published to the PX01/world topic.

1. Connect Pixhawk and Nuc through usbc
2. Ssh into nuc
3. Run: 
    1. ros2 launch mavros px4.launch 
    2. ros2 launch uwb_drone_experiments offboard_square_example.launch.py
4. You should be able to echo (and they should be the same):
    1. /mavros/vision_pose/pose
    2. /mavros/local_position/pose
5. On QGroundControl:
    1. Connect transmitter.
    2. Change mode to Offboard.
    3. Arm the vehicle.


