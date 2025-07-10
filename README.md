# smallRbotArm

This project is based on https://www.youtube.com/watch?v=oFCUw1pXlnA&list=RDCMUCcgqJ1blFKqbC2bWGY4Opmg&index=5

Here are the changes i made and functionalities added:

    2. an python UI built by chatGPT to control the robot, also plot
        the robot movemnt in real-time. it is amazing!

    3. implement trajectory planning (linear function with parabolic blends) learned from https://www.coursera.org/learn/robotics1/lecture/EddnO/7-3-gui-ji-gui-hua-shi-li-fang-fa-er
    I set 2 via points in my program between init and final points.
    the robot arm will move to those points at designated time. this project took me lots of time to make it work and the outcome is acceptable. This project is very cool.

    4. set up an antique 3d camera (xbox 360) above the robot, use it to capture the image of an object on the table, compute the object's coordinate (x, y and z), transfer it into the robot's base frame, move the robot arm to pick it up accroding to the transferred coordinate. 

outstanding issues:
    use g-code decoder to interface btw py and arduino instead of using propiertary 
G90 ; Set absolute coordinates
G28 ; Home all axes
G1 X100 Y100 Z100 A45 B45 C45 F1000 ; Move to position (100, 100, 100) with orientation (45, 45, 45)
In this example, we first set the coordinate system to absolute coordinates using G90. Then we home all axes using G28. Finally, we move the robot arm to position (100, 100, 100) with orientation (45, 45, 45) using G1.

####################################################################################################
    1. pull inverse kinematic and foward kinematic from Simple6Dof_Ver2.ino to python so as to enable the  control the robot arm by sending poses of end-effect to arudino from PC through usb port.

    2. an python UI program mostly built by chatGPT to mainipulate the robot and, at the same time, plot the its movemnt in 3d chart.

    3. Implementing trajectory planning (linear function with parabolic blends) learned from https://www.coursera.org/learn/robotics1/lecture/EddnO/7-3-gui-ji-gui-hua-shi-li-fang-fa-er

    Two via points is set between init and final points.
    the robot arm will move to those points at designated time. It took me lots of time to complete and the outcome is very cool.

    4. set up an antique 3d camera (xbox360) above the robot to capture the image of a red object on the table. compute the object's coordinate wrt the robot's base frame for the robot arm to pick it up.

#####################################################################################
In the Robotics Toolbox for MATLAB, the reach function computes the workspace of a robot and returns the result in units that are consistent with the robot's kinematic model. The units used depend on the specific robot model and can vary depending on the configuration and parameters of the robot.

For example, if the robot is modeled using the standard Denavit-Hartenberg (DH) parameters, the reach function will return the workspace in units of meters. If the robot is modeled using the modified DH (MDH) parameters, the reach function will return the workspace in units of millimeters.

It is important to note that the reach function returns the workspace in the coordinate system defined by the robot's kinematic model, which may not be the same as the world frame or any other external coordinate system. If you need to convert the workspace coordinates to a different unit or coordinate system, you will need to perform a coordinate transformation using the appropriate transformation matrices.

The reach function in the Robotics Toolbox for MATLAB returns a vector of six values representing the reachability of the robot in six degrees of freedom (DOF). Each value in the vector corresponds to the maximum distance that the robot's end-effector can reach along a particular axis of motion.

The six values in the vector represent the reachability along the X, Y, and Z axes of the robot's base frame, as well as the roll, pitch, and yaw axes of the end-effector. By computing the maximum reach along each of these axes, the reach function can determine the overall workspace of the robot in all six degrees of freedom.

It is important to note that the reachability values returned by the reach function are relative to the robot's base frame and are dependent on the specific configuration and parameters of the robot model. Additionally, the actual reachability of the robot may be affected by factors such as joint limits, collisions with obstacles in the environment, or other operational constraints. Therefore, the reachability values returned by the reach function should be used as a rough estimate of the robot's capabilities and should be validated through physical testing and simulation.


