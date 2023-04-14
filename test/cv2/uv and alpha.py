import numpy as np
import cv2

img = cv2.imread('xx.png')
h,w=img[:,:,1].shape
gray=cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

k = np.ones((3, 3))
thr=cv2.threshold(gray, 100, 255, cv2.THRESH_BINARY_INV)
eroded = cv2.erode(thr, k, iterations=3)
dilated = cv2.dilate(eroded, k, iterations=1)
contours, _ = cv2.findContours(
    dilated, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
max_c = max(contours, key=cv2.contourArea)
rect = cv2.minAreaRect(max_c)
u = int(rect[0][0]-w/2)
v = int(h/2-rect[0][1])
f = 600
print(f'alpha, {:1.2f}, u/f*180/np.pi')
cv2.drawContuours(img, [max_c], 0, (255.0, 0), 3)
cv2.imshow('img', img)
cv2.waitKey()
cv2.destroyAllWindows()
