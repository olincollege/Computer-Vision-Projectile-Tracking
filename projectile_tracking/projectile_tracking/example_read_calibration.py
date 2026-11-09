"""
This is an example of how to read the calibration
matrix data outputted by camera_calibration.py

Note that you will need to build and source after
each time you run camera calibration
"""

import yaml
from ament_index_python.packages import get_package_share_directory
import cv2 as cv
import numpy as np

# Find the directory of the yaml file
share_dir = get_package_share_directory("projectile_tracking")
yaml_path = share_dir + '/config/calibration.yaml'

# Open it and read its contents
with open(yaml_path, "r", encoding="utf-8") as file:
    yaml_dict = yaml.safe_load(file)

# Unpack the data returned by cv2.calibrateCamera()
mtx = np.array(yaml_dict["mtx"])
dist = np.array(yaml_dict["dist"])

# Open and prepare the image
img = cv.imread(share_dir + '/photos/_Q1A8026.png')
h,  w = img.shape[:2]
newcameramtx, roi = cv.getOptimalNewCameraMatrix(mtx, dist, (w,h), 1, (w,h))

# undistort
dst = cv.undistort(img, mtx, dist, None, newcameramtx)

# # crop the image
x, y, w, h = roi
dst = dst[y:y+h, x:x+w]
cv.imshow("calibrated image", dst)
cv.waitKey(0)
cv.destroyAllWindows()
