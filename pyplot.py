import numpy as np
import roboticstoolbox as rtb
from roboticstoolbox.backends.PyPlot import PyPlot
# import matplotlib.pyplot as plt
from spatialmath import SE3

a1, a2, a3 = 47.0, 110.0, 26.0
d1, d4, d6 = 133.0, 117.50, 28.0

dh_table = [
    rtb.RevoluteDH(d=d1, a=a1, alpha=-np.pi/2),    # joint 1
    rtb.RevoluteDH(d=0, a=a2, alpha=0, offset=-np.pi/2),             # joint 2
    rtb.RevoluteDH(d=0, a=a3, alpha=-np.pi/2),      # joint 3
    rtb.RevoluteDH(d=d4, a=0, alpha=np.pi/2),       # joint 4
    rtb.RevoluteDH(d=0, a=0, alpha=-np.pi/2),       # joint 5
    rtb.RevoluteDH(d=d6, a=0, alpha=0)              # joint 6
]
frames = [
    SE3.Tx(0)*SE3.Ty(0)*SE3.Tz(20),
    SE3.Rz(np.pi)*SE3.Ry(-np.pi/2)*SE3.Rz(0)
]

robot = rtb.DHRobot(dh_table, name='SmallRobotArm',
                base=frames[0], tool=frames[-1])

#robot = rtb.models.DH.Puma560()                  # instantiate robot model
print(robot.dhunique)
robot.isspherical()
# link values (offset: d, len: a, join angle:theta)
# twist:alpha, joint offset: offset, max reach: reach
print(robot.d)

#pyplot = rtb.backends.PyPlot()
pyplot = PyPlot()
pyplot.launch()
pyplot.add(robot)
robot.q = [1,2,3,4,5,6] #q
# robot.plot(robot.q)

traj = rtb.jtraj(robot.qz, robot.qr, 100)
print(traj.q.shape)
rtb.xplot(traj.q)

robotUrdf = rtb.models.URDF.Puma560()
print(robotUrdf)

print(robot.qr)
#T = robot.fkine([0.1, 0.2, 0.3, 0.4, 0.5, 0.6])  # forward kinematics
# computes the fk for the robot @joint confiugration, q
T = robot.fkine(robot.qz, end='robot_hand')
print(T)
sol = robot.ikine_LM(T)  # inverse kinematics
robot.ikine_a(T, config="lun")        # analytic inverse kinematics
print(sol)
robot.plot((1,2,3,4,5,6), 'pyplot', loop=True)
