from math import radians
from time import sleep, perf_counter
import logging
import pandas as pd
import numpy as np
# import serial_class as com
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


def main() -> None:
    DOF = 6
    bTrajectory = False
    np.set_printoptions(precision=2, suppress=True)
    # r6=distance btw axis6 and end-effector
    std_dh_params = np.array([
        [radians(-90), a1, d1], [0, a2, 0], [radians(-90), a3, 0],
        [radians(90), 0, d4], [radians(-90), 0, 0], [0, 0, d6]
    ])
    # tool frame. this is for generating Tc6
    # Xtf = robot.Point(0.0, 0.0, 50.0, 180.0, -90.0, 0.0)
    # tool frame transformation matrix
    # tc6 = Xtf.pos2tran(), 50mm is the distance btw frame6 to end-effector
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
    smallRobotArm = robot.SmallRbtArm(std_dh_params)
    # print(smallRobotArm.dhTbl)

    # Print the transformation matrix in SE(3) format
    print("dhTbl =\n" + np.array2string(smallRobotArm.dhTbl, separator=', ',
                                        formatter={'float': '{: 0.2f}'.format}))
    # init serial
    #conn = com.SerialPort()
    # there must be a delay here
    sleep(1)
    smallRobotArm.enable()

    # motors are disabled in arduino's setup()
    # conn.ser.write(b"en\n")
    # sleep(.5)
    # conn.ser.write(b"rst\n")
    # sleep(.5)

    # j = smallRobotArm.ik(initPose)
    smallRobotArm.moveTo(initPose)
    #j = [0, 0, 0, 0, 90, 0]
    # conn.send2Arduino('j', j, bWaitAck=True)

    sleep(2)
    
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
    poses = np.array(
        [
            # current is(90,180,-90), after rotating the current orientation by 35 deg about z axis
            # the new zyz is (35,180,-55)
            # [2, 164.5+70,   0.0+25,    241.0-130,  145.0,   90.0, 180.0],
            [0,  264.5+19,   70.0+20,    60,        0.0,    0.0,    35.0],
            [8,  264.5+19,   70.0+20,    90,        0.0,    0.0,    35.0],
            [20, 264.5-120,  70.0+60,    350.0,     0.0,    -60.0,  0.0],
            [24, 264.5-120,  70.0+100,   355.0,     0.0,    -60.0,  0.0],], dtype=float)
    
    # give time col to joints
    joints = poses

    if bTrajectory == False:
        for pose in poses:
            _, *p = pose
            smallRobotArm.moveTo(p)
            # col 0 are time data
            #j = smallRobotArm.ik(pose[1: 7])
            #conn.send2Arduino('j', j, bWaitAck=True)
        '''
        T = SE3(pose[1],  pose[2], pose[3]) * \
            SE3.Rz(np.radians(
                pose[4])) * SE3.Ry(np.radians(pose[5])) * SE3.Rz(np.radians(pose[6]))
        iks = smRobot.ikine_LM(T)
        q = np.degrees(iks.q)
        # joints[i, 1:7] = q.round(2)
        '''

    ###################################################################
    else:
        for (i, pose) in enumerate(poses, start=0):
            # col 0 are time data
            _, *p = pose
            joints[i, 1: 7] = smallRobotArm.ik(p)

        # display easily readable ik resutls on the screen
        J = pd.DataFrame(joints, columns=[
            'ti', 'q1', 'q2', 'q3', 'q4', 'q5', 'q6'])
        print(J.round(2))

        print('--- Start trajectory planning ---')
        (v, a) = pt.planTraj(joints)
        (totalPoints, _) = np.shape(joints)
        logging.info(v)
        logging.info(a)
        ts = joints[:, 0]
        # get rid of time col
        js = joints[:, 1: 7]
        sleep(1)
        # get time from col 0 of p

        start_time = curr_time = perf_counter()
        Xx = np.zeros([6], dtype=np.float32)
        # ts[-1] last one element
        while curr_time - start_time <= ts[-1]:
            dt = curr_time - start_time
            print('Time elasped:{time:.4f}'.format(time=dt))

            for col in range(DOF):
                if dt >= ts[0] and dt <= ts[0] + 0.5:
                    Xx[col] = js[0, col]+eq.eq1(dt, v[0, col], a[0, col])
                elif dt > ts[0] + 0.5 and dt <= ts[1] - 0.25:
                    Xx[col] = js[0, col] + eq.eq2(dt, v[1, col])
                elif dt > ts[1] - 0.25 and dt <= ts[1] + 0.25:
                    Xx[col] = js[0, col] + \
                        eq.eq3(dt, ts[1], v[1, col], a[1, col])
                elif dt > ts[1] + 0.25 and dt <= ts[2] - 0.25:
                    Xx[col] = js[1, col] + eq.eq4(dt-ts[1], v[2, col])
                elif dt > ts[2] - 0.25 and dt <= ts[2] + 0.25:
                    Xx[col] = js[1, col] + eq.eq5(dt, ts, v[2, col], a[2, col])
                elif dt > ts[2] + 0.25 and dt <= ts[totalPoints - 1] - 0.5:
                    Xx[col] = js[2, col] + eq.eq6(dt-ts[2], v[3, col])
                elif dt > ts[totalPoints - 1] - 0.5 and dt <= ts[totalPoints - 1]:
                    Xx[col] = js[2, col] + \
                        eq.eq7(dt, ts, v[3, col], a[3, col], totalPoints)

            # ask arduino to run goTractory(Xx)
            cmd = {'header': 'm', 'joint_angle': Xx, 'ack': False}
            smallRobotArm.conn.send2Arduino(cmd)
            # must be a delay here. ack takes too long causing discontinued arm movement.
            sleep(1/100)
            curr_time = perf_counter()

    sleep(2)
    # this is home pos but axis 6 is not moved back to its origin pos. maybe tool frame isn't considered
    smallRobotArm.moveTo([47.96, 0.0, 268.02, 180, 94.61, 180.0])
    smallRobotArm.disable()
    # a way to terminate thread
    smallRobotArm.conn.disconnect()  
    print('THREAD TERMINATED!')

if __name__ == "__main__":
    main()
