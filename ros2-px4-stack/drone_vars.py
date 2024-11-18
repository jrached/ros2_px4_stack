import numpy as np

VEHICLE_NAMES = ["5quad", "4quad", "gdzilla"]
VEHICLE_NAMES = ["4quad", "5quad", "gdzilla"]
VEHICLE_NAMES = ["gdzilla", "4quad", "5quad"]
# VEHICLE_NAMES = ["gdzilla", "4quad"]
# VEHICLE_NAMES = ["4quad", "gdzilla"]
# VEHICLE_NAMES = ["4quad", "5quad"]
ROS_NAMESPACES = ["fishberg_" + name for name in VEHICLE_NAMES]
TRAJ_FPATH = "/tmp/trajectories.csv"


# VICON system
_VICON_FRONT_LEFT = np.array([-4.089985859870076, -3.4755896395095656])
_VICON_BACK_LEFT = np.array([-3.8534914376884357, 4.372449129187796])
_VICON_BACK_RIGHT = np.array([4.335863252397187, 4.398618398846084])
_VICON_FRONT_RIGHT = np.array([4.580628484813711, -3.5106417586260115])

# Optitrack west space (x, y, z)
# Front-left = -16.78, -4.26, 0.89
# Back-left = -16.91, 4.28, 1.00
# Back-right = -6.60, 4.74, 1.02
# Front-right = -7.22, -4.18, 0.81
optitrack_west_left_max_x = max(-16.78, -16.91)
optitrack_west_front_max_y = max(-4.26, -4.18)
optitrack_west_right_min_x = min(-6.60, -7.22)
optitrack_west_back_min_y = min(4.28, 4.74)
OPTITRACK_WEST_FRONT_LEFT = np.array([optitrack_west_left_max_x, optitrack_west_front_max_y])
OPTITRACK_WEST_BACK_RIGHT = np.array([optitrack_west_right_min_x, optitrack_west_back_min_y])

# Optitrack east space (x, y, z)
# Front-left = -4.92, -4.32, 0.92
# Back-left = -5.25, 4.60, 1.04
# Back-right = 3.83, 4.42, 1.09
# Front-right = 3.98, -4.56, 0.94
optitrack_east_left_max_x = max(-4.92, -5.25)
optitrack_east_front_max_y = max(-4.32, -4.56)
optitrack_east_right_min_x = min(3.83, 3.98)
optitrack_east_back_min_y = min(4.60, 4.42)
OPTITRACK_EAST_FRONT_LEFT = np.array([optitrack_east_left_max_x, optitrack_east_front_max_y])
OPTITRACK_EAST_BACK_RIGHT = np.array([optitrack_east_right_min_x, optitrack_east_back_min_y])


# Optitrack full space: may need to adjust the y to account for curtain cutoff
# between the spaces -- not if the curtain is pulled back!
unlimited_front_left = OPTITRACK_WEST_FRONT_LEFT
unlimited_back_right = OPTITRACK_EAST_BACK_RIGHT
OPTITRACK_FULL_FRONT_LEFT = unlimited_front_left
OPTITRACK_FULL_BACK_RIGHT = unlimited_back_right

# Mocap space options
mocap_options = ["vicon", "optitrack_full", "optitrack_west", "optitrack_east"]
MOCAP_MODE = "optitrack_full"
HIGHBAY_FRONT_LEFT: np.ndarray
HIGHBAY_BACK_RIGHT: np.ndarray
if MOCAP_MODE == "vicon":
    HIGHBAY_FRONT_LEFT = _VICON_FRONT_LEFT
    HIGHBAY_BACK_RIGHT = _VICON_BACK_RIGHT
elif MOCAP_MODE == "optitrack_full":
    HIGHBAY_FRONT_LEFT = OPTITRACK_FULL_FRONT_LEFT
    HIGHBAY_BACK_RIGHT = OPTITRACK_FULL_BACK_RIGHT
elif MOCAP_MODE == "optitrack_east":
    HIGHBAY_FRONT_LEFT = OPTITRACK_EAST_FRONT_LEFT
    HIGHBAY_BACK_RIGHT = OPTITRACK_EAST_BACK_RIGHT
elif MOCAP_MODE == "optitrack_west":
    HIGHBAY_FRONT_LEFT = OPTITRACK_WEST_FRONT_LEFT
    HIGHBAY_BACK_RIGHT = OPTITRACK_WEST_BACK_RIGHT
