import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/austin/ros2_ws/src/Computer-Vision-Projectile-Tracking/install/projectile_tracking'
