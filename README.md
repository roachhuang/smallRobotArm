# smallRbotArm
This project is based on https://www.youtube.com/watch?v=oFCUw1pXlnA&list=RDCMUCcgqJ1blFKqbC2bWGY4Opmg&index=5

Here are the changes i made to make it more flexible:
    1. pull inverse kinematic and foward kinematic from Simple6Dof_Ver2.ino to python so that we can control the robot arm from PC by sending poses of end-effect to arudino through usb port.
    
    2. implement trajectory planning (linear function with parabolic blends) learned from https://www.coursera.org/learn/robotics1/lecture/EddnO/7-3-gui-ji-gui-hua-shi-li-fang-fa-er
    I set 2 via points in my program between init and final points.
    the robot arm will move to those points at designated time. this took me lots of time to make it work and the outcome is very cool.