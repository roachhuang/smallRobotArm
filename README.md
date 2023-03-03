# smallRbotArm
This project is based on https://www.youtube.com/watch?v=oFCUw1pXlnA&list=RDCMUCcgqJ1blFKqbC2bWGY4Opmg&index=5

Here are the changes i made to make it more flexible:
    1. pull inverse kinematic and foward kinematic from Simple6Dof_Ver2.ino to python so that we can control the robot arm from PC by sending poses of end-effect to arudino through usb port.

    4. an python UI built by chatGPT to control the robot, also plot
        the robot movemnt in real-time. it is amazing!

    3. implement trajectory planning (linear function with parabolic blends) learned from https://www.coursera.org/learn/robotics1/lecture/EddnO/7-3-gui-ji-gui-hua-shi-li-fang-fa-er
    I set 2 via points in my program between init and final points.
    the robot arm will move to those points at designated time. this took me lots of time to make it work and the outcome is very cool.

    4. add a 3d camera on top of the robot, capture an image to get an object's coordinate wrt the world frame for the robot arm to pick it up dynamically.

In the Robotics Toolbox for MATLAB, the reach function computes the workspace of a robot and returns the result in units that are consistent with the robot's kinematic model. The units used depend on the specific robot model and can vary depending on the configuration and parameters of the robot.

For example, if the robot is modeled using the standard Denavit-Hartenberg (DH) parameters, the reach function will return the workspace in units of meters. If the robot is modeled using the modified DH (MDH) parameters, the reach function will return the workspace in units of millimeters.

It is important to note that the reach function returns the workspace in the coordinate system defined by the robot's kinematic model, which may not be the same as the world frame or any other external coordinate system. If you need to convert the workspace coordinates to a different unit or coordinate system, you will need to perform a coordinate transformation using the appropriate transformation matrices.

The reach function in the Robotics Toolbox for MATLAB returns a vector of six values representing the reachability of the robot in six degrees of freedom (DOF). Each value in the vector corresponds to the maximum distance that the robot's end-effector can reach along a particular axis of motion.

The six values in the vector represent the reachability along the X, Y, and Z axes of the robot's base frame, as well as the roll, pitch, and yaw axes of the end-effector. By computing the maximum reach along each of these axes, the reach function can determine the overall workspace of the robot in all six degrees of freedom.

It is important to note that the reachability values returned by the reach function are relative to the robot's base frame and are dependent on the specific configuration and parameters of the robot model. Additionally, the actual reachability of the robot may be affected by factors such as joint limits, collisions with obstacles in the environment, or other operational constraints. Therefore, the reachability values returned by the reach function should be used as a rough estimate of the robot's capabilities and should be validated through physical testing and simulation.








