#!/usr/bin/env python
"""
https://openkinect.org/wiki/Getting_Started

sudo apt-get install cython3
sudo apt-get install python3-dev python3-pip git
git clone https://github.com/OpenKinect/libfreenect.git
cd libfreenect/wrappers/python
python3 setup.py build_ext --inplace
sudo python3 setup.py install

https://github.com/amiller/libfreenect-goodies

"""

from freenect import sync_get_depth as get_depth, sync_get_video as get_video
import freenect
import cv2
import numpy as np


# Define callback functions for RGB and depth data
def rgb_callback(devPtr, rgb, timestamp):
    # Convert RGB data to OpenCV image format
    rgb_image = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)

    # Display RGB image
    cv2.imshow('RGB', rgb_image)
    cv2.waitKey(1)


def depth_callback(devPtr, depth, timestamp):
    # Convert depth data to numpy array format
    depth_array = np.array(depth)

    # Normalize depth values to range 0-255
    depth_norm = cv2.normalize(
        depth_array, None, 0, 255, cv2.NORM_MINMAX, dtype=cv2.CV_8UC1)

    # Display depth image
    cv2.imshow('Depth', depth_norm)
    cv2.waitKey(1)


# Initialize Kinect sensor and set callback functions
ctx = freenect.init()
devPtr = freenect.open_device(ctx, 0)

freenect.set_depth_callback(devPtr, depth_callback)
freenect.set_video_callback(devPtr, rgb_callback)

# Start Kinect sensor data acquisition
freenect.start_depth(devPtr)
freenect.start_video(devPtr)

# Wait for user to close window
cv2.waitKey(20)

# Stop Kinect sensor data acquisition

freenect.stop_depth(devPtr)
freenect.stop_video(devPtr)
freenect.close_device(devPtr)
freenect.shutdown(ctx)



def doloop():
    global depth, rgb
    while True:
        # Get a fresh frame
        (depth, _), (rgb, _) = get_depth(), get_video()

        # Build a two panel color image
        d3 = np.dstack((depth, depth, depth)).astype(np.uint8)
        da = np.hstack((d3, rgb))

        # Simple Downsample
        cv2.imshow('both', np.array(da[::2, ::2, ::-1]))
        cv2.waitKey(5)


doloop()

"""
IPython usage:
 ipython
 [1]: run -i demo_freenect
 #<ctrl -c>  (to interrupt the loop)
 [2]: %timeit -n100 get_depth(), get_rgb() # profile the kinect capture

"""
