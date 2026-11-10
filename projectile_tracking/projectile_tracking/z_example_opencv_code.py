import numpy as np
import cv2 as cv
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
import glob
import math

'''image_path = (
    Path(get_package_share_directory("projectile_tracking"))
    / "photos"
    / "_Q1A8031.png"
)'''

share_dir = get_package_share_directory("projectile_tracking")
frame_paths = glob.glob(share_dir + "/photos/*.png")
print(frame_paths[0])
image_path = frame_paths[0]  # Use the first image in the list

scale_down = 0.02
img_unsized = cv.imread(image_path, cv.IMREAD_GRAYSCALE)
img = cv.resize(img_unsized, None, fx=scale_down, fy=scale_down, interpolation=cv.INTER_LINEAR)
cv.imshow('resized_image', img)
cv.waitKey(0)

h, w = img.shape[:2] #Get the height and width of the image


MaxDist = math.hypot(h, w) #Set the minimum distance between detected circles
#print(img)
#cv.imshow('image', img)
#cv.waitKey(0)

assert img is not None, "file could not be read, check with os.path.exists()"
img = cv.medianBlur(img,5)
#cv.imshow('blurred image', img)

cimg = cv.cvtColor(img,cv.COLOR_GRAY2BGR)
#cv.imshow('color image', cimg)
#cv.waitKey(0)
circles = cv.HoughCircles(img,cv.HOUGH_GRADIENT,1,20,
                            param1=50,param2=30,minRadius=0,maxRadius=0)
print(circles)
circles = np.uint16(circles)#np.around(circles))
print(circles)
for i in circles[0,:]:
    # draw the outer circle
    cv.circle(cimg,(i[0],i[1]),i[2],(0,255,0),2)
    # draw the center of the circle
    cv.circle(cimg,(i[0],i[1]),2,(0,0,255),3)
 
cv.imshow('detected circles',cimg)
cv.waitKey(0)
cv.destroyAllWindows()