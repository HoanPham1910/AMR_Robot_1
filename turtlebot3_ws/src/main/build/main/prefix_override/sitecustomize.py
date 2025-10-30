import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/amr/turtlebot3_ws/src/turtlebot3/turtlebot3_bringup/main/install/main'
