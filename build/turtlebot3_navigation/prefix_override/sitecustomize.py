import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/anthony/Desktop/Assignment8/install/turtlebot3_navigation'
