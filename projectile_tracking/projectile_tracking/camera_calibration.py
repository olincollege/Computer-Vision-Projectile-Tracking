import numpy as np
import cv2 as cv
import glob
from ament_index_python.packages import get_package_share_directory
import yaml

# termination criteria
criteria = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 30, 0.001)

# prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
objp = np.zeros((7*10,3), np.float32)
objp[:,:2] = np.mgrid[0:105:15,0:150:15].T.reshape(-1,2)
 
# Arrays to store object points and image points from all the images.
objpoints = [] # 3d point in real world space
imgpoints = [] # 2d points in image plane.

share_dir = get_package_share_directory("projectile_tracking")
images = glob.glob(share_dir + "/calibration_photos/*.png")
 
for fname in images:
    img = cv.imread(fname)
    gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)

    # Find the chess board corners
    ret, corners = cv.findChessboardCorners(gray, (7,10), None)

    # If found, add object points, image points (after refining them)
    if ret == True:
        objpoints.append(objp)

        corners2 = cv.cornerSubPix(gray,corners, (11,11), (-1,-1), criteria)
        imgpoints.append(corners2)

        # Draw and display the corners
        cv.drawChessboardCorners(img, (7,10), corners2, ret)
        cv.imshow('img', img)
        cv.waitKey(500)

cv.destroyAllWindows()

ret, mtx, dist, rvecs, tvecs = cv.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)

yaml_dict = {
    "ret" : ret,
    "mtx" : mtx,
    "dist" : dist,
    "rvecs" : rvecs,
    "tvecs" : tvecs
}

with open('projectile_tracking/config/calibration.yaml', 'w') as file:
    yaml.dump(yaml_dict, file, default_flow_style=False, sort_keys=False)
