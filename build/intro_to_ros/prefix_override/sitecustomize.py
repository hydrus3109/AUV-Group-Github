import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/shamakgowda/auvc_ws/src/intro_to_ros/install/intro_to_ros'
