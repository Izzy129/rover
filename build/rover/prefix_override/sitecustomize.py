import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/baracuda/Desktop/Rice Robotics/workspace/install/rover'
