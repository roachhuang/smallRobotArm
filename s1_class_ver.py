"""_summary_
Measure the position of the target location relative to the table frame,
using a measuring tape, laser range finder, or other measurement tool.

Calculate the position of the target location in the robot's base frame,
by transforming the measured position from the table frame to the robot's
base frame. This can be done using a coordinate transformation matrix
that describes the position and orientation of the table frame in the
robot's base frame.

Determine the desired orientation of the end-effector at the target
location. Depending on your application, this may be a fixed orientation
(e.g. always pointing straight down), or it may be a variable orientation
that depends on the specific task.

Combine the calculated position and orientation into a homogeneous
transformation matrix T, as described in my previous answer.
This matrix represents the pose that you will use as input to the inverse
kinematics algorithm.
    Returns:
        _type_: _description_
"""
from math import radians
from threading import Thread, Event
from time import sleep, perf_counter
# from serial.threaded import ReaderThread, Protocol
import logging
import pandas as pd
import numpy as np
import serial.tools.list_ports
import serial
import equations as eq
import robotarm_class as robot
import plan_traj as pt
from roboticstoolbox import DHRobot, RevoluteDH
from spatialmath import SE3

# Define your DH table parameters
# 50 is distance btw 6th axis and end-effector
a1, a2, a3 = 47.0, 110.0, 26.0
d1, d4, d6 = 133.0, 117.50, 28.0

std_dh_table = [
    RevoluteDH(d=d1, a=a1, alpha=-np.pi/2),    # joint 1
    RevoluteDH(d=0, a=a2, alpha=0, offset=-np.pi/2),             # joint 2
    RevoluteDH(d=0, a=a3, alpha=-np.pi/2),      # joint 3
    RevoluteDH(d=d4, a=0, alpha=np.pi/2),       # joint 4
    RevoluteDH(d=0, a=0, alpha=-np.pi/2),       # joint 5
    RevoluteDH(d=d6, a=0, alpha=0)              # joint 6
]

# Create the robot object with the DH parameters
smRobot = DHRobot(std_dh_table)

# {x, y, z, ZYZ Euler angles}
# Xhome = [164.5, 0.0, 241.0, 90.0, 180.0, -90.0]

event_ok2send = Event()
"""
This module provides functions for manipulating strings.

The functions included in this module are:

- send2Arduino: send robot cmd to arudino at 115200bps
- receiveThread: thread to receive incoming data from arduino
- init_ser: init serial port (auto select port#)
"""

def send2Arduino(ser, header: str, j, bWaitAck: bool):
    """send robot cmd to arduino

    Args:
        ser (_type_): _description_
        header (str): cmd type
        j (float): theta in deg for 6 axes
        bWaitAck (bool): wait for ack from arduino or not
    """
    msg = f'{header}{j[0]:.2f},{j[1]:.2f},{j[2]:.2f},{j[3]:.2f},{j[4]:.2f},{j[5]:.2f}\n'
    ser.write(msg.encode('utf-8'))
    event_ok2send.clear()
    print(msg)
    if bWaitAck is True:
        event_ok2send.wait()
    # while event_ack.is_set() and bWaitAck is True:
    #    pass


def ReceiveThread(ser, event_run):
    """
    input string is retrieved as a byte string, which works
    differently than a standard string. The decode() method is to convert
    the string from a byte string to a standard string.
    """
    while event_run.is_set():
        line = ser.readline().decode('utf-8')
        if len(line) > 0:
            # get rid of the end of characters /n/r
            string = line.rstrip()
            logging.info(string)
            print(string)
            if string == 'ack':
                # receive ack frm arduion meaning it is free now
                event_ok2send.set()


def init_ser():
    """
    Initializes a serial connection to an Arduino board.

    Returns:
        A `serial.Serial` object representing the serial connection, or `None` if no suitable
        serial port is found or if the connection fails.
    """
    ports = list(serial.tools.list_ports.comports())
    ser = None

    if not ports:
        print("No serial ports found.")
        return None

    # Skip over COM1 port, if available
    ports = [p for p in ports if "COM1" not in p.name]

    for port in ports:
        try:
            ser = serial.Serial(port.name, 115200, timeout=.1)
            break
        except (OSError, serial.SerialException):
            print(f"Could not connect to serial port {port.name}")

    if not ser:
        print("Failed to connect to any serial port.")
        return None

    print(f"Connected to serial port {ser.name}")
    return ser


def main() -> None:
    bTrajectory = False
    DOF = 6
    event_run = Event()
    event_run.clear()
    np.set_printoptions(precision=2, suppress=True)
    # r6=distance btw axis6 and end-effector
    r1, r2, r3, = 47.0, 110.0, 26.0
    # d1, d3, d4, d6 = 133.0, 0.0, 117.50, 28.0
    dh_tbl = np.array([
        [radians(-90), r1, d1], [0, r2, 0], [radians(-90), r3, 0],
        [radians(90), 0, d4], [radians(-90), 0, 0], [0, 0, d6]
    ])
    # tool frame. this is for generating Tc6
    # Xtf = robot.Point(0.0, 0.0, 50.0, 180.0, -90.0, 0.0)
    # tool frame transformation matrix
    # tc6 = Xtf.pos2tran()
    tc6 = robot.pose2T([0.0, 0.0, 50.0, 180.0, -90.0, 0.0])
    # orientation for frame 6 is unchanged (x points up, z->front)
    t60 = robot.pose2T([164.5, 0.0, 241.0, 90.0, 180.0, -90])
    # t60 = robot.pose2T([164.5, 0.0, 241.0, 180.0, -90.0, 0])
    tc0 = t60 @ tc6
    T_se3 = SE3(tc0)
    tZ1, tY, tZ2 = T_se3.eul(unit='deg')
    initPose = np.zeros(6)
    initPose[0:3] = tc0[0:3, 3]
    initPose[3:6] = tZ1, tY, tZ2
    # create an instance of the robotarm.
    smallRobotArm = robot.RobotArm(6, dh_tbl)
    # print(smallRobotArm.dhTbl)

    # Print the transformation matrix in SE(3) format
    print("dhTbl =\n" + np.array2string(smallRobotArm.dhTbl, separator=', ',
                                        formatter={'float': '{: 0.2f}'.format}))
    # init serial
    ser = init_ser()
    # ser.reset_input_buffer()
    # ser.reset_output_buffer()
    event_run.set()
    event_ok2send.set()
    t = Thread(target=ReceiveThread, args=[ser, event_run])
    t.start()
    sleep(1)
    # motors are disabled in arduino's setup()
    ser.write(b"en\n")
    sleep(.5)
    ser.write(b"rst\n")
    sleep(.5)
    j = smallRobotArm.ik(initPose)
    # j=[0,0,0,0,90,0]
    send2Arduino(ser, 'j', j, bWaitAck=True)
    '''
    T = SE3(initPose[0],  initPose[1], initPose[2]) * \
        SE3.Rz(np.radians(
            initPose[3])) * SE3.Ry(np.radians(initPose[4])) * SE3.Rz(np.radians(initPose[5]))
    j = smRobot.ikine_LM(T)
    q = np.degrees(j.q)
    # send2Arduino(ser, 'j', q.round(2), bWaitAck=True)
    '''

    """
    for curPos are all zero
    after fk for [0, 78.51, -73,9, 0, -1.14, 0], computed manually from arduino goHome code.
    home postion = [301.85, 0.0, 168.6, -180, -84.0, 0]
    """
    # end-effector's position and orientation
    poses = np.array([
        # current is(90,180,-90), after rotating the current orientation by 35 deg about z axis
        # the new zyz is (35,180,-55)
        # [2, 164.5+70,   0.0+25,    241.0-130,  145.0,   90.0, 180.0],
        [2, 264.5,   70.0+25,    40,           0.0,    0.0,    35.0],
        [2, 264.5,   70.0+25,    80,           0.0,    0.0,    35.0],
        [0, 264.5-150,  70.0+100,   300.0,     0.0,    -60.0,  0.0],
        [0, 264.5-150,  70.0+130,   300.0,     0.0,    -60.0,  0.0],
        # [0, 264.5-100,  100.0+80,   241.0+110,  180.0,  60.0,  180.0],
        # [0, 264.5-120,  100.0+120,   241.0+110,  180.0,  60.0,   180.0],

        # [6, 301.85,      0.0, 168.6, -180.0, -84.0, 0.0],
        # [1, 271.8,    0.0,      142.9,  -180,        0.0,       0],
        # [16, 192.5, -30.0, 241.0,  90.0, 180.0, 0.0],
        # [21, 47.96, 0.0, 288.02, 180, 94.61, 180.0],

        [21, 51.98, 0, 218.2, 0.0, 0.0, 180.0],

    ], dtype=float)
    # give time col to joints
    joints = poses

    for (i, pose) in enumerate(poses, start=0):
        # col 0 are time data
        start_time = perf_counter()
        joints[i, 1: 7] = smallRobotArm.ik(pose[1: 7])
        '''
        T = SE3(pose[1],  pose[2], pose[3]) * \
            SE3.Rz(np.radians(
                pose[4])) * SE3.Ry(np.radians(pose[5])) * SE3.Rz(np.radians(pose[6]))
        iks = smRobot.ikine_LM(T)
        q = np.degrees(iks.q)
        # joints[i, 1:7] = q.round(2)
        '''
        end_time = perf_counter()
        print(
            f'It took {end_time- start_time: 0.2f} second(s) to complete IK.')

    # display easily readable ik resutls on the screen
    J = pd.DataFrame(joints, columns=[
                     'ti', 'q1', 'q2', 'q3', 'q4', 'q5', 'q6'])
    print(J.round(2))

    if bTrajectory is False:
        for joint in joints[:, 1:7]:
            send2Arduino(ser, 'j', joint, bWaitAck=True)
            # print('send cmd to arduino')
    ###################################################################
    else:
        print('--- Start trajectory planning ---')
        (v, a) = pt.planTraj(joints)
        (totalPoints, _) = np.shape(joints)
        logging.info(v)
        logging.info(a)
        ts = joints[:, 0]
        # get rid of time col
        joints = joints[:, 1: 7]
        # send2Arduino(ser, 'J', joints)
        # send2Arduino(ser, 'V', v)
        # send2Arduino(ser, 'A', a)
        sleep(1)
        # ser.write(b"T\n")
        # zero = np.zeros([6], dtype=str)
        # get time from col 0 of p

        start_time = curr_time = perf_counter()
        Xx = np.zeros([6], dtype=float)
        # ts[-1] last one element
        while curr_time - start_time <= ts[-1]:
            dt = curr_time - start_time
            # print('Time elasped:{time:.3f}'.format(time=t))
            for col in range(DOF):
                if dt >= ts[0] and dt <= ts[0] + 0.5:
                    Xx[col] = joints[0, col]+eq.eq1(dt, v, a, col)
                elif dt > ts[0] + 0.5 and dt <= ts[1] - 0.25:
                    Xx[col] = joints[0, col] + eq.eq2(dt, v, col)
                elif dt > ts[1] - 0.25 and dt <= ts[1] + 0.25:
                    Xx[col] = joints[0, col] + eq.eq3(dt, ts, v, a, col)
                elif dt > ts[1] + 0.25 and dt <= ts[2] - 0.25:
                    Xx[col] = joints[1, col] + eq.eq4(dt, ts, v, col)
                elif dt > ts[2] - 0.25 and dt <= ts[2] + 0.25:
                    Xx[col] = joints[1, col] + eq.eq5(dt, ts, v, a, col)
                elif dt > ts[2] + 0.25 and dt <= ts[totalPoints - 1] - 0.5:
                    Xx[col] = joints[2, col] + eq.eq6(dt, ts, v, col)
                elif dt > ts[totalPoints - 1] - 0.5 and dt <= ts[totalPoints - 1]:
                    Xx[col] = joints[2, col] + \
                        eq.eq7(dt, ts, v, a, col, totalPoints)

            # event.set()

            # msg='m'+','.join(Xx.astype(str))
            # msg = 'm%.2f\t%.2f\t%.2f\t%.2f\t%.2f\t%.2f\n' % (Xx[0], Xx[1], Xx[2], Xx[3], Xx[4], Xx[5])
            # msg = f'm{Xx[0]:.2f},{Xx[1]:.2f},{Xx[2]:.2f},{Xx[3]:.2f},{Xx[4]:.2f},{Xx[5]:.2f}\n'
            # ask arduino to run goTractory(Xx)
            # ser.write(msg.encode())
            send2Arduino(ser, 'm', Xx, bWaitAck=False)
            # while event.is_set():
            #    pass  # sleep(.1)

            # must be a delay here. ack takes too long causing discontinued arm movement.
            sleep(1/1000000)
            curr_time = perf_counter()
    sleep(1)
    # ser.write(b"dis\n")
    # find a way to terminate thread
    event_run.clear()
    t.join()
    ser.close()
    print('THREAD TERMINATED!')


if __name__ == "__main__":
    main()
