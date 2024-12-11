import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/juan/mavros_ws/src/ros2_px4_stack/install/ros2_px4_stack'
