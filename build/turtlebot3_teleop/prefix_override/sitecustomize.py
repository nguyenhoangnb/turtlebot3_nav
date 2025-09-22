import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/hoang/turtlebot3_ws/src/install/turtlebot3_teleop'
