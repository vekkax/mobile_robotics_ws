import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/vekkaz/mobile_robotics_ws/install/scan_pkg'
