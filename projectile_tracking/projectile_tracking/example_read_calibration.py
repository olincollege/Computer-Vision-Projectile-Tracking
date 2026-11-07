"""
This is an example of how to read the calibration
matrix data outputted by camera_calibration.py

Note that you will need to build and source after
each time you run camera calibration
"""

import yaml
from ament_index_python.packages import get_package_share_directory

# Find the directory of the yaml file
share_dir = get_package_share_directory("projectile_tracking")
yaml_path = share_dir + '/config/calibration.yaml'

# Open it and read its contents
with open(yaml_path, "r", encoding="utf-8") as file:
    yaml_dict = yaml.safe_load(file)

# Unpack the data returned by cv2.calibrateCamera()
ret = yaml_dict["ret"]
mtx = yaml_dict["mtx"]
dist = yaml_dict["dist"]
rvecs = yaml_dict["rvecs"]
tvecs = yaml_dict["tvecs"]
