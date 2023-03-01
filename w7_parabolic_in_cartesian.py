from math import atan2, sqrt
import craig as cg
import numpy as np
import pieper as pp
import pandas as pd
import plan_traj as pt

def main():
    np.set_printoptions(precision=4, suppress=True)
    dh_tbl = np.array([[0, 0, 0], [np.deg2rad(-90), -30, 0], [0, 340, 0],
                       [np.deg2rad(-90), -40, 338], [np.deg2rad(90), 0, 0],
                       [np.deg2rad(-90), 0, 0]])

    cg.setDhTbl(dh_tbl)

    # time, x, y, z, tx, ty, tz(wrt world frame), end-effector 在各點的 position and 姿態
    p = np.array([
        [0, 550, 270, 19.5, 0, 0, 35],
        [2, 550, 270, 79.5, 0, 0, 35],  # viapoint1
        [6, 330, 372, 367, 0, -60, 0],  # viapoint2
        [9, 330, 472, 367, 0, -60, 0],
    ])

    # duration
    #print (t)
    col_names = ['ti', 'xi', 'yi', 'zi', 'qx', 'qy', 'qz']
    row_names = ['p0', 'p1', 'p2', 'pf']
    P = pd.DataFrame(p, columns=col_names, index=row_names)
    print(P)
    print(' ')

    Tcup_6 = np.array([[0, 0, 1, 0], [0, -1, 0, 0], [1, 0, 0, 206],
                       [0, 0, 0, 1]])
    tc_0 = []
    # pg 6, compute each point's tc_0 transformation matrix based on cartesion space. range: 0~totalPoints-1
    tf = np.empty(shape=[4, 4])  #p0 to pf
    t6_0 = []

    DOF = 6
    totalPoints, num_cols = p.shape
    segs = totalPoints - 1
    # substract 頭, 尾
    viaPoints = totalPoints - 2
    for i in range(totalPoints):
        tx, ty, tz = np.deg2rad(p[i, 4:7])
        # combine rot and translation vector into transformation matrix
        tf[:3, :3] = cg.Rot('x', tx) @ cg.Rot('y', ty) @ cg.Rot('z', tz)  # rotation matrix
        tf[:3, 3] = p[i, 1:4]  # x,y,z
        tf[3, :] = [0, 0, 0, 1]
        tc_0.append(tf)
        # get t6_0 for each point
        t6_0 = tc_0[i] @ np.linalg.inv(Tcup_6)
        # replace p with new values
        # get euler angles from rotation matrix
        p[i, 4:7] = cg.rotationMatrixToEulerAngles(t6_0)
        p[i, 1:4] = t6_0[0:3, 3].T
    # from t6_0 to get P6_0_org 在各點的position and 姿態.
    print(P)
    print(' ')

    pt.planTraj(p)

    # plot 建立並繪出各DOF 在每個時間區段軌跡
    # linear/parabolic 共7段 （每段parabolic curve 時間設定為0.5s）

    # ik p0 ~ pf 所有的點的 theta (too many points, not good for cpu loading)
    # plot thetas to time for the 6 axes （以此theats 對 time 的關係來control motors)
    # final verification - FK T6_0=t1_0 @ ... t6_5
    # plot to simulate

if __name__ == "__main__":
    main()
