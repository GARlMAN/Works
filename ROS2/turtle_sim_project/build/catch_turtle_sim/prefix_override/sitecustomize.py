import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/gari/Documents/ROS2/turtle_sim_project/install/catch_turtle_sim'
