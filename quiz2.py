import craig as cg
import numpy as np
from math import asin, acos, sin, cos, pi, atan2

np.set_printoptions(precision=4, suppress=True)


def q1():
    # fix angle roation so the rotation multiplication is in reverse order
    y = cg.Rot('y', 'a')
    x = cg.Rot('x', 'b')
    print('an1:', x @ y)


def q2():
    print('an2:', cg.Rot('z', 'a') @ cg.Rot('x', 'b'))


def q3():
    pb = np.array([3, 1, 5])
    r = cg.Rot('z', np.deg2rad(30)) @ cg.Rot('x', np.deg2rad(45))
    print('an3:', r @ pb.T)


def q4():
    # fix angle
    print(cg.Rot('z', 'g') @ cg.Rot('y', 'b') @ cg.Rot('x', 'a'))
    y_rad = asin(0.7071)
    print('beta:', np.rad2deg(y_rad))
    print('alp:', np.rad2deg(asin(0.5 / cos(y_rad))))
    print('gamma:', np.rad2deg(asin(0.6124 / cos(y_rad))))


def q5():
    #euler angle
    R = np.array([[-0.5736, -0.8192, 0], [-0.8192, 0.5736, 0], [0, 0, -1]])
    print(R)
    # print('eluer:', cg.Rot('z', 'a') @ cg.Rot('y', 'b') @ cg.Rot('z', 'g'))
    eul1 = atan2(R.item(1, 2), R.item(0, 2))
    sp = sin(eul1)
    cp = cos(eul1)
    eul2 = atan2(cp * R.item(0, 2) + sp * R.item(1, 2), R.item(2, 2))
    eul3 = atan2(-sp * R.item(0, 0) + cp * R.item(1, 0),
               -sp * R.item(0, 1) + cp * R.item(1, 1))

    print("phi =", np.rad2deg(eul1))
    print("theta =", np.rad2deg(eul2))
    print("psi =", np.rad2deg(eul3))


def q6():
    move = np.array([0, -4, 4])
    ap2 = np.array([1, -3, -1])
    print('an6:', cg.Rot('z', np.deg2rad(-45)) @ ap2.T + move.T)


q1()
q2()
q3()
q4()
q5()
q6()