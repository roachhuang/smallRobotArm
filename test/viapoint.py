from matplotlib.pyplot import plot
import roboticstoolbox as rtb
import numpy as np
from spatialmath import SE3

#first = np.array([10, 20])
#last = np.array([30, 10])
first = SE3.Rand()
last = SE3.Rand()
viaPoints = np.array([
    [-5, 5, 45],
    [2, 3, 30],
    [2, -3, 0]])
# joint traj
#tg = rtb.jtraj(first, last, 50)

# cartesian traj
tg = rtb.ctraj(first, last, 50)

tg = rtb.mstraj(viaPoints, dt = 1, tacc = 0.5, tsegment = [2, 4, 9], q0 = [-4, 0, 120], verbose = True)

print(tg)
tg.plot()
#rtb.xplot(tg)