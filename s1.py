#!/usr/bin/env python3
from ik_smallrobotarm import InverseK
from math import radians
import std_dh_tbl as stdDh
import serial
from time import sleep, perf_counter
# from serial.threaded import ReaderThread, Protocol
from threading import Thread, Event
import pandas as pd
import numpy as np
import logging
import plan_traj as pt
import serial.tools.list_ports

np.set_printoptions(precision=4, suppress=True)

'''
p = np.array([
    [0, 264, 50.0, 19.0, 0, 0, 35],
    [2, 264, 50.0, 79.5, 0, 0, 35],  # viapoint1
    [6, 164, 100.0, 141, 0, -60, 0],  # viapoint2
    [9, 164, 141.0, 141, 0, -60, 0],
])
'''
# {x, y, z, ZYZ Euler angles}
Xhome = [164.5, 0.0, 241.0, 90.0, 180.0, -90.0]

# j = np.empty(shape=[4, 7])
j = []
#bArmBusy = False
event = Event()
event.clear()

def ReceiveThread(ser):
    while True:
        line = ser.readline()
        if len(line) > 0:
            string = line.decode('utf-8').rstrip()
            logging.info(string)
            print(string)
            if string == 'ack':
                # receive ack frm arduion meaning it is free now
                event.clear()
            '''
            splitPacket = string.split('\t')
            # gets ik in radian from arduino
            if(splitPacket[0] == 'i'):
                j.append(splitPacket[1: 7])
                # j[0, 1:7] = splitPacket[1:7]
                # row = row+1
                # print(J)
                logging.info(j)
            '''


ports = serial.tools.list_ports.comports()
# print([port.name for port in ports])
for port in ports:
    try:
        ser = serial.Serial(port.name, 115200, timeout=.1)
    except (OSError, serialException):
        pass
# ser.flushInput()
# ser.flushOutput()
# ser.reset_input_buffer()
t = Thread(target=ReceiveThread, args=[ser])
t.start()

ser.write(b"Hello from Raspberry Pi!\n")
sleep(1)

points = np.array([
    [0, 164.5, 0.0, 141.0, 90.0, 180.0, -90.0],
    [2, 164.5+14.7, 35.4, 141.0, 90.0, 180.0, -90.0],
    [6, 164.5+50.0, 50.0, 141.0, 90.0, 180.0, -90.0],
    [9, 164.5+85.3, 35.4, 141.0, 90.0, 180.0, -90.0]
])

r1, r2, r3 = 47.0, 110.0, 26.0
d1, d3, d4, d6 = 133.0, 0.0, 117.50, 28.0

dh_tbl = np.array([[radians(-90), r1, d1], [0, r2, 0], [radians(-90), r3, 0],
                   [radians(90), 0, d4], [radians(-90), 0, 0],
                   [0, 0, d6]])

stdDh.setDhTbl(dh_tbl)

# # of row on the p list
# (totalPoints, _) = points.shape

col_names = ['ti', 'xi', 'yi', 'zi', 'qx', 'qy', 'qz']
row_names = ['p0', 'p1', 'p2', 'pf']
P = pd.DataFrame(points, columns=col_names, index=row_names)
print(P)
print(' ')

'''
    # speed_message = 's %f %f\r' % (self._left_wheel_speed_, self._right_wheel_speed_)
    msg = f'po{pos[0]}\t{pos[1]}\t{pos[2]}\t{pos[3]}\t{pos[4]}\t{pos[5]}'+'\n'
    ser.write(b"ik\n")
    print("President {}: {}".format(num, name))
'''
###################################################################
# preserve time col
# do IK and send the results to arduino
joints = points
for (i, point) in enumerate(points, start=0):
    # col 0 are time data
    start_time = perf_counter()
    joints[i, 1: 7] = InverseK(point[1: 7])
    end_time = perf_counter()
    print(f'It took {end_time- start_time: 0.2f} second(s) to complete.')
    msg = 'j%.2f\t%.2f\t%.2f\t%.2f\t%.2f\t%.2f\n' % (joints[i, 1], joints[i, 2],
                                                     joints[i, 3], joints[i, 4], joints[i, 5], joints[i, 6])

    ser.write(msg.encode())
    event.set()
    print(msg)
    # print(event.is_set())
    while event.is_set():
        pass  # sleep(.1)

# display easily readable ik resutls on the screen
col_names = ['ti', 'q1', 'q2', 'q3', 'q4', 'q5', 'q6']
J = pd.DataFrame(joints, columns=col_names, index=row_names)
print(J.round(2))

###################################################################
print('--- Start trajectory planning ---')
# (v, a) = pt.planTraj(joints)
logging.info(v)
logging.info(a)
# take 4 decimal points and convert them to string array for serial sending
v = np.around(v, decimals=4)
v = np.array(v, dtype=str)
a = np.around(a, decimals=4)
a = np.array(a, dtype=str)

zero = np.zeros([6], dtype=str)

# t.join() it is a while loop won't terminate unless break by such as kb interrupt

"""
runSpeed() which simply runs a stepper at a constant speed set by setSpeed() and limited by setMaxSpeed().
But AccelStepper can do much more! As the name implies, it can accelerate a stepper as well.
This is done using run().
The run() function will accelerate a motor to the speed set by setMaxSpeed() at the acceleration rate set by
setAcceleration().
It will step the motor for the number of steps set by moveTo(), called the target. When the motor nears the target,
AccelStepper will decelerate the motor to a smooth stop at the target.
"""
'''
# start sending v and a to robot arm
# v0, a0 - parabolic segment. v0: init speed. to accelerate the motor using run()
# run ignore setSpeed's value, The run() function will accelerate until the maximum speed as set is reached.
# setMaxSpeed to higher so, it won't never reach.
sendVelAcc(v[0, :], a[0, :])
time.sleep(0.5)  # 0, 2, 6, 9

# v0, 0  # linear - no acc; keep running at a constant speed at v0 using runSpeed()
sendVelAcc(v[1, :], zero)
time.sleep(1.25)    # duration, dt.

# v1, a1 - parabolic segment. to accel init speed @v1 for 0.5s
sendVelAcc(v[1, :], a[1, :])
time.sleep(0.5)

# v2, 0 - linear segment
sendVelAcc(v[2, :], zero)
time.sleep(3.5)

# v2, a2 - parabolic segment
sendVelAcc(v[2, :], a[2, :])
time.sleep(0.5)

# v3, 0 - linear segment
sendVelAcc(v[3, :], zero)
time.sleep(2.25)

# v3, a3 - parabolic segment
sendVelAcc(v[3, :], a[3, :])
time.sleep(0.5)

ser.close()
'''


def sendCmd(v, a):
    msg = 'vel'
    msg += '\t'.join(v)+'\n'
    ser.write(msg.encode())

    msg = 'acc'
    msg += '\t'.join(a)+'\n'
    ser.write(msg.encode())

    ser.write(b"go\n")
