import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/johan/ros2_ws/src/text_to_cmd_vel_pkg/install/text_to_cmd_vel_pkg'
