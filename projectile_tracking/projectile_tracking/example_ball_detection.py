import numpy as np
import cv2 as cv
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
import glob
import math
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

# Set minDist for Hough Circle Transform based on diagonal image size
h, w = photo.shape[:2] #Get the height and width of the image
minDist = math.hypot(h, w) + 1 #Set the minimum distance between detected circles

grey_image = cv.cvtColor(photo, cv.COLOR_BGR2GRAY)
plt.subplot(121), plt.imshow(grey_image, cmap='gray'), plt.axis("off"), plt.title("Grayscale")
plt.subplot(122), plt.hist(grey_image.ravel(),256,[0,256],color='k'), plt.title("Gray Histogram")
plt.show()

# RGB Histograms
for i,c in enumerate(('r','g','b')):
    plt.plot(cv.calcHist([photo],[i],None,[256],[0,256]), color=c)
plt.title("RGB Histograms"), plt.xlabel("Intensity"), plt.ylabel("Frequency")
plt.show()
# Apply Hough Circle Transform to detect circles in the image
circles = cv.HoughCircles(grey_image, cv.HOUGH_GRADIENT_ALT, dp = 1.5, minDist = 1, param1 = 300, param2 = 0.6, minRadius = 0, maxRadius = 0) #Detect circles in the image
#circles = cv.HoughCircles(grey_image,cv.HOUGH_GRADIENT,1,20, param1=50,param2=30,minRadius=0,maxRadius=0)
print(circles)
for x, y, radius in circles[0]:
    # draw the circle
    cv.circle(photo, (int(x), int(y)), int(radius), (0, 255, 0), thickness = 2)

    detected_balls.append((x, y, radius))  # Add the detected ball position and radius to list

    # Display the detected circles on the image
    cv.imshow('detected circles', photo)


    # Keep the window open and close when key pressed
    cv.waitKey(0)
    cv.destroyAllWindows()