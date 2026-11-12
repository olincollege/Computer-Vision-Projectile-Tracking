import numpy as np
import cv2 as cv
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
import glob
import math
import yaml
import matplotlib
import matplotlib.pyplot as plt

'''image_path = (
    Path(get_package_share_directory("projectile_tracking"))
    / "photos"
    / "_Q1A8031.png"
)'''

share_dir = get_package_share_directory("projectile_tracking")
frame_paths = glob.glob(share_dir + "/photos/*.png")
print(frame_paths[0])
image_path = frame_paths[0]  # Use the first image in the list
scale_down = 0.13
img_unsized = cv.imread(image_path, cv.IMREAD_COLOR)
photo = cv.resize(img_unsized, None, fx=scale_down, fy=scale_down, interpolation=cv.INTER_LINEAR)
#photo = cv.medianBlur(photo_unfiltered, 5)


detected_balls = []

share_dir = get_package_share_directory("projectile_tracking")
yaml_path = share_dir + '/config/color_mask.yaml'

with open(yaml_path, "r", encoding="utf-8") as file:
    color_range = yaml.safe_load(file)

lower = np.array([
                color_range["hMin"],
                color_range["sMin"],
                color_range["vMin"],
            ])

upper = np.array([
                color_range["hMax"],
                color_range["sMax"],
                color_range["vMax"],
            ])

            # Convert BGR to HSV, then produce a binary mask of pixels
            # whose color falls within the configured bounds
hsv = cv.cvtColor(photo, cv.COLOR_BGR2HSV)
color_mask = cv.inRange(hsv, lower, upper)
masked_photo = cv.bitwise_and(photo, photo, mask=color_mask)

# Set minDist for Hough Circle Transform based on diagonal image size
h, w = photo.shape[:2] #Get the height and width of the image
minDist = math.hypot(h, w) + 1 #Set the minimum distance between detected circles

grey_image = cv.cvtColor(masked_photo, cv.COLOR_BGR2GRAY)

# Apply Hough Circle Transform to detect circles in the image
circles = cv.HoughCircles(grey_image, cv.HOUGH_GRADIENT_ALT, dp = 1.5, minDist = minDist, param1 = 300, param2 = 0.6, minRadius = 0, maxRadius = 0) #Detect circles in the image
#circles = cv.HoughCircles(grey_image,cv.HOUGH_GRADIENT,1,20, param1=50,param2=30,minRadius=0,maxRadius=0)
for x, y, radius in circles[0]:
    # draw the circle
    cv.circle(photo, (int(x), int(y)), int(radius), (0, 255, 0), thickness = 2)

    detected_balls.append((x, y, radius))  # Add the detected ball position and radius to list

    # Display the detected circles on the image
    cv.imshow('detected circles', photo)


    # Keep the window open and close when key pressed
    cv.waitKey(0)
    cv.destroyAllWindows()