import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from scipy.interpolate import CubicSpline
# import plotly.express as px
# plt.ion()

# 開始規劃 trajectory. Degree Of Freedom.
DOF = 6


def planTraj(p):
     # in 0, 0.5s. col[1~3]: x, y, z
    def eq1(t, col):
        dt = t - 0
        v0 = v[0, col]
        a0 = a[0, col]
        # col+1 coz time on the 1st col. parabloic (二次多項式)
        # Xeq1(t) = x0+v0*dt+1/2*at^2, here we replace d with theta
        return p[0, col + 1] + v0 * dt + 1 / 2 * a0 * dt**2

    # in 0.5, 1.75
    def eq2(t, col):
        # Xeq2(t)=x0+v1*dt.
        dt = t - 0.25
        v1 = v[1, col]
        # linear segment.
        return p[0, col + 1] + v1 * dt

    # in 1.75, 2.25
    def eq3(t, col):
        v1 = v[1, col]
        a1 = a[1, col]
        dt1 = t - 0.25
        dt2 = t - (ts[1] - 0.25)
        return p[0, col + 1] + v1 * dt1 + 1 / 2 * a1 * dt2**2

    # in 2.25, 5.75
    def eq4(t, col):
        dt = t - ts[1]
        v2 = v[2, col]
        return p[1, col + 1] + v2 * dt

    # 5.75, 6.25
    def eq5(t, col):
        dt1 = t - ts[1]
        dt2 = t - (ts[2] - 0.25)
        v2 = v[2, col]
        a2 = a[2, col]
        return p[1, col + 1] + v2 * dt1 + 1 / 2 * a2 * dt2**2

    # 6.25, 8.5
    def eq6(t, col):
        dt = t - ts[2]
        v3 = v[3, col]
        return p[2, col + 1] + v3 * dt

    # col - 0:x, 1:y, 2:theta
    # 8.5 ~9s
    def eq7(t, col):
        dt1 = t - ts[2]
        dt2 = t - (ts[totalPoints - 1] - 0.5)
        v3 = v[3, col]
        a3 = a[3, col]
        return p[2, col + 1] + v3 * dt1 + 1 / 2 * a3 * dt2**2
    
    """input array p contains timing and joint angles at each points"""
    (totalPoints, num_cols) = np.shape(p)
    # 有 totalPoints 的2次段, 内有totalPoints-1 的linear 段
    # 幾條綫段
    segs = totalPoints - 1
    # num_of_v = segs + 2  # 頭+尾 2 個, vini and vfin
    # substract 頭, 尾
    # viaPoints = totalPoints - 2
    # get difference between points on each axis for calculating vel.
    diff = np.diff(p, axis=0)

    v1s = np.array([])
    v2s = np.array([])
    v3s = np.array([])
    """
    V2k(k=x,y,theta or q1~q6) = dof2-dof1/4s-2s
    头尾区段: (时间段稍微内移，保证二次项圆滑）
    """
    # 對P6_0 在各點的pos and 姿態, compute vel, set 每段parabolic func 區間長0.5s
    durationOfPara = 0.5
    # starting from 1 coz 0 is col of time
    for col in range(1, num_cols):
        v1s = np.append(v1s, diff[0, col] / (diff[0, 0] - durationOfPara / 2))
        v2s = np.append(v2s, diff[1, col] / (diff[1, 0]))
        v3s = np.append(v3s, diff[2, col] / (diff[2, 0] - durationOfPara / 2))
    #
    # 頭尾補0, coz, init and final points' velocity are 0
    v = np.array(
        [np.zeros(num_cols - 1), v1s, v2s, v3s, np.zeros(num_cols - 1)], dtype=float
    )
    # (m, _) = np.shape(v)
    # v = np.insert(v, (0, m), 0, axis=0)

    col_names = [str(c) for c in range(num_cols - 1)]
    # col_names = ['x', 'y', 'z', 'qx', 'qy', 'qz(deg/s)']

    row_names = ["v0", "v1", "v2", "v3", "vf"]
    V = pd.DataFrame(v, columns=col_names, index=row_names)
    print(V)
    # set 2次段在0.5s 完成加速, 讓在0.5s 完成和linear vel 一樣的vel.
    # a0=v1-v0/0.5, a1=v2-v1/0.5, a2=v3-v2/0.5, a3-vf-v3/0.5
    a = np.diff(v, axis=0) / durationOfPara
    row_names = ["a0", "a1", "a2", "af"]
    A = pd.DataFrame(a, columns=col_names, index=row_names)
    print(A)

    # get time from col 0 of p
    ts = p[:, 0]

    # 0s ~ final second, step:0.1
    timeAxis = np.arange(ts[0], ts[-1], 0.1)
    inputPoints = [[] * 1 for _ in range(num_cols - 1)]   
    
    # col - 0~2, denote x, y or theta data
    # inputPoints=np.empty((3, 90))
    fig_col = 3
    fig_row = int(DOF / fig_col)
    # fig = plt.figure(facecolor='lightblue', figsize=(fig_row*3, fig_col*3))
    fig = plt.figure(figsize=(fig_row * 3, fig_col * 3))
    fig.subplots_adjust(wspace=0.25, hspace=0.25)
    plt.title("Robot arm joints angle changes over time", fontsize=20, color="blue")
    plt.grid()
    
    '''        
    # Use CubicSpline interpolation
    splines = [CubicSpline(ts, p[:, col + 1]) for col in range(num_cols - 1)]
    for col in range(num_cols - 1):
        inputPoints[col] = splines[col](timeAxis)
    
    for col in range(num_cols - 1):
        ax = fig.add_subplot(fig_row, fig_col, col + 1)
        ax.set_xlabel('Time in sec.')
        ax.set_ylabel('Degree')
        plt.plot(timeAxis, splines[col](timeAxis), 'r')
    plt.show()
    '''   
    
    for col in range(num_cols - 1):
        for t in timeAxis:
            if t >= ts[0] and t <= ts[0] + 0.5:
                inputPoints[col].append(eq1(t, col))
            elif t > ts[0] + 0.5 and t <= ts[1] - 0.25:
                inputPoints[col].append(eq2(t, col))
            elif t > ts[1] - 0.25 and t <= ts[1] + 0.25:
                inputPoints[col].append(eq3(t, col))
            elif t > ts[1] + 0.25 and t <= ts[2] - 0.25:
                inputPoints[col].append(eq4(t, col))
            elif t > ts[2] - 0.25 and t <= ts[2] + 0.25:
                inputPoints[col].append(eq5(t, col))
            elif t > ts[2] + 0.25 and t <= ts[totalPoints - 1] - 0.5:
                inputPoints[col].append(eq6(t, col))
            elif t > ts[totalPoints - 1] - 0.5 and t <= ts[totalPoints - 1]:
                inputPoints[col].append(eq7(t, col))
        # this fig has 1 row, 3 col in one page

        # (rows, columns, panel number)
        ax = fig.add_subplot(fig_row, fig_col, col + 1)
        ax.set_xlabel("Time in sec.")
        ax.set_ylabel("Degree")
        # plt.subplot(fig_row, fig_col, col + 1, title=f'{col+1} to time')
        plt.plot(timeAxis, inputPoints[col], "r")
    plt.show()    
    
    fig = plt.figure(figsize=(9, 9))
    plt.title(
        "Trajectory planning with Linear functions & parabolic blending",
        fontsize=20,
        color="blue",
    )
    ax = fig.add_subplot(1, 1, 1, projection="3d")
    # in the case of time, x, y and theta
    ax.set_xlabel("x-axis")
    ax.set_ylabel("y-axis")
    if num_cols == 4:
        ax.set_zlabel("time-axis")
        # ax.plot3D(inputPoints[0], inputPoints[1], timeAxis, 'r')
        ax.scatter3D(inputPoints[0], inputPoints[1], timeAxis, c=timeAxis)
    else:
        ax.set_zlabel("z-axis")
        # data = pd.DataFrame({'X':inputPoints[0], 'Y':inputPoints[1], 'Z':inputPoints[2],'Time':timeAxis})
        # fig = px.scatter_3d(data, x = "X", y = "Y", z = "Z", hover_data = ["Time"])

        # ax.plot3D(inputPoints[0], inputPoints[1], inputPoints[2], color='r', linestyle='dotted')
        ax.scatter3D(inputPoints[0], inputPoints[1], inputPoints[2], color="g")

    # plt.tight_layout()
    plt.show()
    # return vel and acc arrays
    return (v, a)
