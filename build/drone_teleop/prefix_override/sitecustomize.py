import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/galdrux/gazebo_projects/drone_ws/install/drone_teleop'
