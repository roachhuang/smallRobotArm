import roboticstoolbox as rtb
import numpy as np
import spatialmath as sm
import swift
import spatialgeometry as sg

env=swift.Swift()
# ask swift to run in realtime or asap (good for demo and video recording, etc.)
env.launch(realtime=True)

# init the model
robot = rtb.models.Panda()
print(robot.urdf_string)
# set the joint angles (coordinates) of the robot to be the rq config
# q.d: joint vel
robot.q = robot.qr
# 1st joint to rotate at 0.1 rad/s (last arg is end effector)
robot.qd = [0.1, 0, 0, 0, 0, 0, 0.2]

# add robot to swift, make it 50% transparent
env.add(robot, robot_alpha=1.0, collision_alpha=0.5)

# 100 times
#for _ in range(100):
    # dt 0.05s
    # everytime calls step the simulator uses joint vel to calc where it will be in 50ms
#    env.step(0.05)

# set goal pose. calc fk using its current joint coordinates
# calc is the end effector pose in the world frame, then offset this pose
# in the x direction by 20cm, y direction by 20cm and z by 35cm.
# Tep is the desired pose
Tep = robot.fkine(robot.q)* sm.SE3.Tx(0.2)*sm.SE3.Ty(0.2)*sm.SE3.Tz(0.35)
# to viuize in swift by putting some axes in env
# define a set of axes these are right-handed coordinates they have a len of 10cm and their pose to be tep
axes = sg.Axes(length=0.1, base=Tep)
env.add(axes)

# flag
arrived = False
# time step
dt = 0.01
while not arrived:
    # v is a 6 vector representing the spatial err btw one pose & another pose
    # robot.q is the end effector pose calcuated using fk,
    # p_servo will calc the xyz pos & rpy pos btw these 2 poses & apply a gain
    # threshold is for when it is deemed to arrive so can set arrive flag to True
    v, arrived = rtb.p_servo(robot.fkine(robot.q), Tep, gain=1, threshold=0.01)
    # calc jacobian of the robot in the end effector frame coz v is the spatial err in the end of vector frame
    # pass in robot.q, the robot's current joint coordinates
    J = robot.jacobe(robot.q)
    # then i ca work out the desired joint vel of the robot by doing pseduo invse of the jacobianb and multiply that by desired end eff vel
    robot.qd = np.linalg.pinv(J) @ v
    # step the env
    env.step(dt)

# stop the browser tab from closing
env.hold()

# visualise the robot in its zero joint config
# you can change it to qr, which is a much nicer joint config
# block=True stops the program from exiting
# robot.plot(robot.qr, block=True)
#q.z is zero joint configuration & q.r is some predefined ready joint config