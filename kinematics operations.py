import roboticstoolbox as rtb
from spatialmath import SE3

robot = rtb.models.DH.Puma560()
print(robot)
T = robot.fkine([0.1,0.2,0.3,0.4,0.5,0.6])  # forward kinematics
print(T)

sol=robot.ikine(T)
print(sol.q)
robot.maniplty(sol.q)
robot.jacob0(sol.q)

robot.plot(robot.qz, vellipse=True)
qt = rtb.jtraj(robot.qr, robot.qz, 50)
robot.plot(qt.q, dt=100, movie='puma_sitting.gif')
