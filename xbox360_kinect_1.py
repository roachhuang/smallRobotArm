import cv2
import numpy as np

"""_summary_
This code uses a similar approach to the previous code to isolate the object in the image, but also estimates the distance to the object based on its area, and returns the (x, y, z) coordinates of the object. Note that the depth threshold and the camera parameters (575.5 and (320, 240)) may need to be adjusted for your specific Kinect setup.
    Returns:
        _type_: _description_
    """
# Set up the Kinect
kinect = cv2.VideoCapture(cv2.CAP_OPENNI)

# Define the dimensions of the object (in meters)
object_width = 0.1
object_height = 0.1

# Define the depth threshold for isolating the object
depth_threshold = 500


def calculate_position():
    # Get the depth data and convert it to a grayscale image
    ret, depth = kinect.read()
    depth = cv2.convertScaleAbs(depth, alpha=0.05)
    depth = cv2.cvtColor(depth, cv2.COLOR_BGR2GRAY)

    # Apply the depth threshold to isolate the object
    _, thresh = cv2.threshold(depth, depth_threshold, 255, cv2.THRESH_BINARY)

    # Find the contours of the object
    contours, _ = cv2.findContours(
        thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    if len(contours) > 0:
        # Find the contour with the largest area
        contour = max(contours, key=cv2.contourArea)

        # Calculate the center of the contour
        moments = cv2.moments(contour)
        cx = int(moments['m10'] / moments['m00'])
        cy = int(moments['m01'] / moments['m00'])

        # Estimate the distance to the object based on its area
        object_area = cv2.contourArea(contour)
        distance = (object_width * object_height) / object_area

        # Convert the pixel coordinates to meters
        x = (cx - 320) * distance / 575.5
        y = (cy - 240) * distance / 575.5
        z = distance

        return (x, y, z)

    else:
        return None


# Main loop
while True:
    position = calculate_position()

    if position is not None:
        print(f"Object position: {position}")

    if cv2.waitKey(1) == 27:
        break

kinect.release()
cv2.destroyAllWindows()
