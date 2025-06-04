# ros2_px4_stack
Mavros interface for ACL hardware experiments 

# Software setup
To install the necessary software for the Dynus planner, follow the instructions in this repo: 
[Dynus Installation](https://github.com/jrached/dynus_setup)


# Running Dynus

Note: This software uses lidar for state estimation and expects an initial position in a global reference frame (from say a motion capture system) in order to align the coordinate frames of the agents. 

1. On a vehicle, from the root of this project, run: 
    ```
    python3 scripts/tmux/dynus_tmux.py 
    ```
2. Arm the vehicle.
3. Then source the workspace and run: 
    ```
    ros2 run ros2_px4_stack there_and_back.py 1.0 3.0 2.0 
    ```

    Where {x, y, z} = {1.0, 3.0, 2.0} is an arbitrary global position goal.


