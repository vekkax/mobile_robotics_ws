import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/vekkaz/mobile_robotics_ws/install/time_to_collision'
