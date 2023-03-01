from math import atan2, sqrt, radians
import craig as cg
import numpy as np
import pieper as pp
import pandas as pd
import plan_traj as pt

def main():
    np.set_printoptions(precision=4, suppress=True)
    dh_tbl = np.array([[0, 0, 0], [radians(-90), -30, 0], [0, 340, 0],
                       [radians(-90), -40, 338], [radians(90), 0, 0],
                       [radians(-90), 0, 0]])

    cg.setDhTbl(dh_tbl)

    # 從機械手臂的Frame {0}座標系來看，杯子的中心（Frame {C}原點）在不同時間點的位置及姿態分別在下表列出。
    # time, x, y, z, tx, ty, tz(wrt world frame)
    p = np.array([
        [0, 550, 270, 19.5, 0, 0, 35],
        [2, 550, 270, 79.5, 0, 0, 35],  # viapoint1
        [6, 330, 372, 367.0, 0, -60, 0],  # viapoint2
        [9, 330, 472, 367.0, 0, -60, 0],
    ])

    # duration
    # print (t)
    col_names = ['ti', 'xi', 'yi', 'zi', 'qx', 'qy', 'qz']
    row_names = ['p0', 'p1', 'p2', 'pf']
    P = pd.DataFrame(p, columns=col_names, index=row_names)
    print(P)
    print(' ')

    Tcup_6 = np.array([[0, 0, 1, 0], [0, -1, 0, 0], [1, 0, 0, 206],
                       [0, 0, 0, 1]])
    tc_0 = []
    # pg 6, compute each point's tc_0 transformation matrix based on cartesion space. range: 0~totalPoints-1
    tf = np.empty(shape=[4, 4])  # p0 to pf
    t6_0 = []
    totalPoints, num_cols = p.shape
    segs = totalPoints - 1
    # substract 頭, 尾
    viaPoints = totalPoints - 2
    DOF = 6

    for i in range(totalPoints):
        (tx, ty, tz) = np.deg2rad(p[i, 4:7])
        # combine rot and translation vector into transformation matrix
        tf[:3, :3] = cg.Rot('x', tx) @ cg.Rot('y',
                                              ty) @ cg.Rot('z', tz)  # rotation matrix
        tf[:3, 3] = p[i, 1:4]  # x,y,z
        tf[3, :] = [0, 0, 0, 1]
        tc_0.append(tf)
        # get t6_0 for each point
        t6_0 = tc_0[i] @ np.linalg.inv(Tcup_6)
        # replace p with ik's result - thetas
        p[i, 1:7] = np.rad2deg(pp.pieper(t6_0))
        # fk to see if q1-q6 computed by ik are correct
        # +0.0 to fix -0 in array, decimals=1 for fixing allclose returns false if decimals=2
        fk_t6_0 = np.around(cg.fk_6axes(np.deg2rad(p[i, 1:7])), decimals=1)+0.0
        print(f'fk_t6_0: {fk_t6_0}')
        assert np.allclose(np.around(t6_0, decimals=1), fk_t6_0)
        # assert np.allclose(t6_0.astype(np.float64), fk_t6_0.astype(np.float64))

    col_names = ['ti', 'q1', 'q2', 'q3', 'q4', 'q5', 'q6']
    P = pd.DataFrame(p, columns=col_names, index=row_names)
    print(P.round(2))

    print('--- Start trajectory planning ---')
    pt.planTraj(p)

    # plot 建立並繪出各DOF 在每個時間區段軌跡
    # linear/parabolic 共7段 （每段parabolic curve 時間設定為0.5s）

    # ik p0 ~ pf 所有的點
    # FK to xyz space
    # plt simulation


if __name__ == "__main__":
    main()
