import craig as cg
import numpy as np
import pieper as pp
import pandas as pd
# from sympy import Matrix, Symbol, Eq, solve, sin, cos

def main():
    np.set_printoptions(precision=4, suppress=True)
    dh_tbl = np.array([[0, 0, 0], [np.deg2rad(-90), -30, 0], [0, 340, 0],
                       [np.deg2rad(-90), -40, 338], [np.deg2rad(90), 0, 0],
                       [np.deg2rad(-90), 0, 0]])

    cg.setDhTbl(dh_tbl)

    viaPoint = 2
    # time, x, y, z, tx, ty, tz
    C = np.array([
        [0, 550, 270, 19.5, 0, 0, 35],
        [2, 550, 270, 79.5, 0, 0, 35],
        [6, 330, 372, 367, 0, -60, 0],
        [9, 330, 472, 367, 0, -60, 0],
    ])
    dt1 = C[1, 0] - C[0, 0]
    dt2 = C[2, 0] - C[1, 0]
    dt3 = C[3, 0] - C[2, 0]
    totalPoints, num_cols = C.shape
    segs = totalPoints - 1
    # substract 頭, 尾
    viaPoints = totalPoints - 2
    DOF=6

    Tcup_6 = np.array([[0, 0, 1, 0], [0, -1, 0, 0], [1, 0, 0, 206],
                       [0, 0, 0, 1]])

    T = np.array([[1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                  [1, dt1, dt1**2, dt1**3, 0, 0, 0, 0, 0, 0, 0, 0],
                  [0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0],
                  [0, 0, 0, 0, 1, dt2, dt2**2, dt2**3, 0, 0, 0, 0],
                  [0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0],
                  [0, 0, 0, 0, 0, 0, 0, 0, 1, dt3, dt3**2, dt3**3],
                  [0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                  [0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 2 * dt3, 3 * (dt3**2)],
                  [0, 1, 2 * dt1, 3 * (dt1**2), 0, -1, 0, 0, 0, 0, 0, 0],
                  [0, 0, 2, 6 * dt1, 0, 0, -2, 0, 0, 0, 0, 0],
                  [0, 0, 0, 0, 0, 1, 2 * dt2, 3 * (dt2**2), 0, -1, 0, 0],
                  [0, 0, 0, 0, 0, 0, 2, 6 * dt2, 0, 0, -2, 0]])

    if (np.linalg.det(T) != 0):
        T_inv = np.linalg.inv(T)
    else:
        print('error det == 0; no inverse')
    # print('inv:', T_inv)
    #print(np.matmul(T_inv, C_of_X))

    #C_of_Q = np.ndarray(shape=(0, 3), dtype=float)
    # size of Q = number of points x 6 (DOF)
    qs_p = []
    tc_0=[]
    # transformation matrix
    tf=np.empty(shape=[4,4])
    # Q=np.empty(shape=[4*segs, DOF])
    Q=[]

    # pg 6, compute each point's tc_0 based on cartesion space. range: 0~totalPoints-1
    for p in range(totalPoints):
        tx, ty, tz = np.deg2rad(C[p, 4:7])
        # combine rot and translation vector into transformation matrix
        tf[:3, :3] = cg.Rot('x', tx)@cg.Rot('y', ty)@cg.Rot('z', tz)   # rotation matrix
        tf[:3, 3] = C[p, 1:4]  # x,y,z
        tf[3, :] = [0,0,0,1]
        tc_0.append(tf)
        #print(tc_0[p])
        # convert cartesian space to joint space: 3 segments 0-2s 2-4s 4-9s
        t6_0 = tc_0[p] @ np.linalg.inv(Tcup_6)
        t6_0[:, 3]=[cg.my_sigfig(i, 4) for i in t6_0[:, 3]]
        q1To6 = pp.pieper(t6_0)
        qs_p.append(q1To6)

    # process segments for pos, vel and acc
    for s in range(segs):
        # pos
        Q.append(qs_p[s])
        Q.append(qs_p[s+1])

    Q=np.asarray(Q)
    Q=np.resize(Q, (4*segs, DOF))
    row_names=['p0','p1','p1','p2','p2','pf','v0','vf', 'via1_vel','via1_acc', 'via2_vel','via2_acc']
    col_names=['q1','q2','q3','q4','q5','q6']
    q = pd.DataFrame(Q,columns=col_names, index=row_names)
    print(np.rad2deg(q))

    # 2 dicimal points accroding to 5-6 exm2
    A = (np.round(T_inv @ Q, 3))
    row_names=['a10','a11','a12','a13','a20','a21','a22','a23', 'a30','a31', 'a32','a33']
    col_names=['q1','q2','q3','q4','q5','q6']
    a = pd.DataFrame(A,columns=col_names, index=row_names)

    # make a0~a3 in one row, 0~1s,2~4s,4~9s in one group(3x4), each group represent one theta
    # 3xseg, 3xtheta, 4xai
    #A12x3 = np.transpose(A12x3).reshape(3, 3, 4)
    #A = np.reshape(A, (3, 3, 4))
    # A is now a 3-D array
    # row: theta, col:aij,
    print(a)
    via_vel=np.empty(shape=[viaPoints, DOF])
    via_acc=np.empty(shape=[viaPoints, DOF])
    for v in range(viaPoints):
        # vel & acc continuity before and after each via point
        for j in range(DOF):
            # get current seg's  coefficients
            via_vel[v, j] = A[4*(v+1)+1, j]
            via_acc[v, j] = 2 * A[4*(v+1)+2,j]

    print(np.rad2deg(via_vel))
    print(np.rad2deg(via_acc))

if __name__ == "__main__":
    main()
