import freenect
import numpy as np
import cv2
import open3d as o3d
import matplotlib.pyplot as plt

point = (400, 300)


def show_distance(event, x, y, args, params):
    global point
    point = (x, y)


cv2.namedWindow('rgb')
cv2.setMouseCallback('rgb', show_distance)


# Get depth and RGB frames from Kinect

depth, _ = freenect.sync_get_depth()
bgr, _ = freenect.sync_get_video()
height, width = depth.shape
#cx = int(width/2)
#cy= int(height/2)

rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
hsv_frame = cv2.cvtColor(bgr, cv2.COLOR_RGB2HSV)

# pick pixel value
#pixel_center = hsv_frame[cy, cx]
# print(pixel_center)
#cv2.circle(rgb, (cx, cy), 5, (255,255,255),3)

''' this ranges are bigger
# Define the lower and upper bounds of the color red in the HSV color space        
lower_red = np.array([0, 50, 50], dtype=np.uint8)
upper_red = np.array([10, 255, 255], dtype=np.uint8)
mask1 = cv2.inRange(hsv_frame, lower_red, upper_red)

lower_red = np.array([170, 50, 50])
upper_red = np.array([180, 255, 255])
mask2 = cv2.inRange(hsv_frame, lower_red, upper_red)
# Create a mask that includes both ranges of red color
red_mask = cv2.bitwise_or(mask1, mask2)
'''


low_red = np.array([161, 155, 84])
high_red = np.array([179, 255, 255])
# Red Color
lower_red = np.array([0, 100, 100])
upper_red = np.array([7, 255, 255])

# yellow color
lower_yellow = np.array([25, 100, 100])
upper_yellow = np.array([30, 255, 255])

# green color
lower_green = np.array([40, 70, 80])
upper_green = np.array([70, 255, 255])

# blue color
lower_blue = np.array([90, 60, 0])
upper_blue = np.array([121, 255, 255])

red_mask = cv2.inRange(hsv_frame, lower_red, upper_red)

# Apply the mask to the original image to keep only the red pixels
red = cv2.bitwise_and(rgb, rgb, mask=red_mask)

gray = cv2.cvtColor(red, cv2.COLOR_BGR2GRAY)
blurred = cv2.GaussianBlur(gray, (11, 11), 0)
canny = cv2.Canny(blurred, 30, 150)
(cnts, _) = cv2.findContours(image=canny,
                             mode=cv2.RETR_TREE, method=cv2.CHAIN_APPROX_NONE)


for c in cnts:
    M = cv2.moments(c)
    if M['m00'] != 0:
        cx = int(M['m10']/M['m00'])
        cy = int(M['m01']/M['m00'])
    else:
        cx, cy = 0, 0
    cv2.drawContours(image=bgr, contours=c, contourIdx=-1,
                     color=(0, 255, 0), thickness=2, lineType=cv2.LINE_AA)
    cv2.circle(rgb, (cx, cy), 7, (255, 255, 255), -1)
    cv2.putText(rgb, "center", (cx - 20, cy - 20),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 2)
# show distance for a specific point
distance = depth[point[1], point[0]]

# put the text 20px above the circle
cv2.putText(rgb, '{}mm'.format(distance),
            (point[0], point[1]-20), cv2.FONT_HERSHEY_PLAIN, 2, (0, 0, 0), 2)
# print(distance)
# print('----------')
#pcd_dis = pcd.points(point[1], point[0])

# print(pcd_dis)


cv2.imshow('rgb', rgb)
cv2.imshow('red', red)
cv2.imshow('Canny', canny)

cv2.waitKey(0)
cv2.destroyAllWindows()
