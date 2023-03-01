import math
import threading as th
import time
# from matplotlib import pyplot as plt
import matplotlib.pyplot as plt
import numpy as np
from traj_joint_space import getCoefficients

def cb1():
    #while True:
    print("1st event")

# do your stuff
#sc.enter(10, 1, do_something, (sc,))
def cb2():
    s1Active = False
    print("2nd event")

    # do your stuff
    #sc.enter(10, 1, do_something, (sc,))


"""
s1 = th.Timer(3, cb1)
s2 = th.Timer(5, cb2)
s2.start()
s1.start()

for i in [1, 2, 3, 4, 5, 6]:
    print(i)
    time.sleep(1)
print('end')
"""

L1 = 5
L2 = 3
L3 = 1
x0 = y0 = 0
figno = 1


def doSeg(dt, seg):
    pos = np.array([0, 0, 0], dtype=float)
    vel = np.array([0, 0, 0], dtype=float)
    global figno

    # i = joint number
    for joint in range(3):
        # get current seg's  coefficients
        a = getCoefficients(joint, seg)
        print(a)
        pos[joint] = a[0] + a[1] * dt + a[2] * dt**2 + a[3] * dt**3
        vel[joint] = a[1] + 2 * a[2] * dt + 3 * a[3] * dt**2
        #print('pos, vel', pos[joint], vel[joint])
        # move joint[i]

    x1 = L1 * math.cos(pos[0])
    y1 = L1 * math.sin(pos[0])
    x2 = x1 + L2 * math.cos(pos[0] + pos[1])
    y2 = y1 + L2 * math.sin(pos[0] + pos[1])
    filename = str(figno) + 'jpg'
    figno = figno + 1
    plt.figure()
    plt.plot([x0, x1], [y0, y1])
    plt.plot([x1, x2], [y1, y2])
    plt.xlim([-8, 8])
    plt.ylim([-8, 8])
    # plt.savefig(filename)
    print('---------------')


# Timer starts
starttime = time.time()
lasttime = starttime
lapnum = 1
value = ""
totaltime = 0

#print("Press ENTER for each lap.\nType Q and press ENTER to stop.")
"""
doSeg(0, 0)
doSeg(2, 0)
doSeg(0, 1)
doSeg(2, 1)
doSeg(0, 2)
doSeg(5, 2)
"""
#input()

while True:

    # Input for the ENTER key press
    #value = input()

    # The current lap-time

    # Total time elapsed since the timer started
    totaltime = round((time.time() - starttime), 2)
    if (totaltime >= 0 and totaltime < 2):
        print('in seg0')
        #dt = round((time.time() - lasttime), 3)
        dt = totaltime - 0
        #lasttime = time.time()
        print('dt:', str(dt))
        doSeg(dt, 0)
        time.sleep(0.1)
    elif (totaltime >= 2 and totaltime < 4):
        print('in seg1')
        dt = totaltime - 2
        #print(dt)
        doSeg(dt, 1)
        time.sleep(0.1)
        #lasttime = time.time()
    elif (totaltime >= 4 and totaltime < 9):
        print('in seg2')
        dt = totaltime - 4
        #print(dt)
        doSeg(dt, 2)
        time.sleep(0.1)
        # dt = round((time.time() - lasttime), 2)
        #lasttime = time.time()
    else:
        print('done')
        break
    # Printing the lap number, lap-time, and total time
    #print("Lap No. " + str(lapnum))
    #print("Total Time: " + str(totaltime))
    #print("Lap Time: " + str(laptime))

    #print("*" * 20)

    # Updating the previous total time and lap number
    lasttime = time.time()
    lapnum += 1

print("Exercise complete!")