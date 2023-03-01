import cv2 as cv2
import numpy as np

# Intrinsic camera parameters
fx = 500.0
fy = 500.0
cx = 320.0
cy = 240.0
camera_matrix = np.array([[fx, 0, cx], [0, fy, cy], [0, 0, 1]])

# Distortion coefficients
k1 = 0.1
k2 = 0.01
k3 = 0.001
p1 = 0.0
p2 = 0.0
dist_coeffs = np.array([k1, k2, p1, p2, k3])

# Load the object model
model_points = np.array(
    [[-0.03, 0.03, 0.0], [-0.03, -0.03, 0.0], [0.03, -0.03, 0.0], [0.03, 0.03, 0.0]])
model_points = np.transpose(model_points)

# Initialize the camera
cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

while True:
    # Capture a frame from the camera
    ret, frame = cap.read()

    # Convert the frame to grayscale
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # Detect the object in the frame
    # ...
    # Load the pre-trained classifier for detecting objects
    try:
        object_classifier = cv2.CascadeClassifier('./objects/banana_classifier.xml')
    except Exception as e:
        print(e)
    # Open the video capture device (use 0 for the default camera)
    cap = cv2.VideoCapture(0)
    # Loop through frames in the video stream
    while True:
        # Read a frame from the video stream
        ret, frame = cap.read()

        # Convert the frame to grayscale
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # Detect objects in the frame
        objects = object_classifier.detectMultiScale(gray)

        # Draw a rectangle around each detected object
        for (x, y, w, h) in objects:
            cv2.rectangle(frame, (x, y), (x+w, y+h), (0, 255, 0), 2)

        # Display the frame with detected objects
        cv2.imshow('Object Detection', frame)

        # Wait for a key press and exit if 'q' is pressed
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    # Release the video capture device and close the window
    cap.release()
    cv2.destroyAllWindows()




#=====================================================
# Compute the pose of the object
# ...

# Draw the pose on the frame
# ...
"""
# Show the frame
cv2.imshow('frame', frame)
if cv2.waitKey(1) & 0xFF == ord('q'):
    break

# Release the camera and close the window
cap.release()
cv2.destroyAllWindows()
"""