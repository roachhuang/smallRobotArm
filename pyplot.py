import roboticstoolbox as rtb
import matplotlib.pyplot as plt

robot = rtb.models.DH.Puma560()                  # instantiate robot model
print(robot.dhunique)
robot.isspherical()
# link values (offset: d, len: a, join angle:theta)
# twist:alpha, joint offset: offset, max reach: reach
print(robot.d)
#pyplot = rtb.backends.PyPlot()
#pyplot.launch()
#pyplot.add(robot)
#robot.q = q
#robot.step()


traj = rtb.jtraj(robot.qz, robot.qr, 100)
traj.q.shape
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
robot.plot((1,2,3,4), 'pyplot', loop=True)
