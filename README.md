# Small Robot Arm

This project is based on https://www.youtube.com/watch?v=oFCUw1pXlnA&list=RDCMUCcgqJ1blFKqbC2bWGY4Opmg&index=5

## Features and Functionalities

    2. an python UI built by chatGPT to control the robot, also plot
        the robot movemnt in real-time. it is amazing!

    3. implement trajectory planning (linear function with parabolic blends) learned from https://www.coursera.org/learn/robotics1/lecture/EddnO/7-3-gui-ji-gui-hua-shi-li-fang-fa-er
    I set 2 via points in my program between init and final points.
    the robot arm will move to those points at designated time. this project took me lots of time to make it work and the outcome is acceptable. This project is very cool.

    4. set up an antique 3d camera (xbox 360) above the robot, use it to capture the image of an object on the table, compute the object's coordinate (x, y and z), transfer it into the robot's base frame, move the robot arm to pick it up accroding to the transferred coordinate. 

## Outstanding Issues

Use G-code decoder to interface between Python and Arduino instead of using proprietary protocol:

```gcode
G90 ; Set absolute coordinates
G28 ; Home all axes
G1 X100 Y100 Z100 A45 B45 C45 F1000 ; Move to position (100, 100, 100) with orientation (45, 45, 45)
```

In this example, we first set the coordinate system to absolute coordinates using G90. Then we home all axes using G28. Finally, we move the robot arm to position (100, 100, 100) with orientation (45, 45, 45) using G1.

## Implementation Details

1. Pulled inverse kinematics and forward kinematics from Simple6Dof_Ver2.ino to Python to enable control of the robot arm by sending end-effector poses from PC to Arduino through USB port.

2. Created a Python UI program to manipulate the robot and simultaneously plot its movement in a 3D chart.

3. Implemented trajectory planning (linear function with parabolic blends) learned from [this Coursera course](https://www.coursera.org/learn/robotics1/lecture/EddnO/7-3-gui-ji-gui-hua-shi-li-fang-fa-er).
   - Set two via points between initial and final points
   - The robot arm moves to these points at designated times

4. Set up an Xbox 360 3D camera above the robot to:
   - Capture images of objects on the table
   - Compute object coordinates (x, y, z)
   - Transform coordinates to the robot's base frame
   - Move the robot arm to pick up objects based on these coordinates

## Robot Workspace and Reachability

In the Robotics Toolbox for MATLAB, the reach function computes the workspace of a robot and returns the result in units that are consistent with the robot's kinematic model. The units used depend on the specific robot model and can vary depending on the configuration and parameters of the robot.

For example, if the robot is modeled using the standard Denavit-Hartenberg (DH) parameters, the reach function will return the workspace in units of meters. If the robot is modeled using the modified DH (MDH) parameters, the reach function will return the workspace in units of millimeters.

It is important to note that the reach function returns the workspace in the coordinate system defined by the robot's kinematic model, which may not be the same as the world frame or any other external coordinate system. If you need to convert the workspace coordinates to a different unit or coordinate system, you will need to perform a coordinate transformation using the appropriate transformation matrices.

The reach function in the Robotics Toolbox for MATLAB returns a vector of six values representing the reachability of the robot in six degrees of freedom (DOF). Each value in the vector corresponds to the maximum distance that the robot's end-effector can reach along a particular axis of motion.

The six values in the vector represent the reachability along the X, Y, and Z axes of the robot's base frame, as well as the roll, pitch, and yaw axes of the end-effector. By computing the maximum reach along each of these axes, the reach function can determine the overall workspace of the robot in all six degrees of freedom.

It is important to note that the reachability values returned by the reach function are relative to the robot's base frame and are dependent on the specific configuration and parameters of the robot model. Additionally, the actual reachability of the robot may be affected by factors such as joint limits, collisions with obstacles in the environment, or other operational constraints. Therefore, the reachability values returned by the reach function should be used as a rough estimate of the robot's capabilities and should be validated through physical testing and simulation.

## Recent Updates

### Signal Handler Module

A new signal handler module has been added to provide graceful shutdown when Ctrl+C is pressed. This ensures that:

1. The robot arm returns to a safe home position
2. Any active threads are properly joined
3. The program exits cleanly

To use the signal handler in your scripts:

```python
from robot_tools.misc.signal_handler import setup_signal_handler

# Create your robot arm instance
robot_arm = SmallRbtArm(std_dh_params)

# Set up the signal handler
setup_signal_handler(robot_arm)

# Optional: Add a custom cleanup function
def custom_cleanup():
    print("Performing custom cleanup...")

setup_signal_handler(robot_arm, custom_cleanup)
```

See the example in `examples/signal_handler_example.py` for a complete demonstration.
