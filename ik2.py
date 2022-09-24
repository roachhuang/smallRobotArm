from math import acos, atan2, sqrt
# delta unit 6 part3 / professor Xiao https://youtu.be/rGQjiOUZhEI
"""image.png"""
# j0 perpendicular to j1; j1 & j2 are parallel
# if jo 沒有偏差值
q1 = atan2(xc, yc)
#sol2:
q1 = pi+atan2(xc,yc)

"""
get R3-0 first
cal q4, q5, q6
R6-3(q4,q5,q6)=(R3-0)T *R
R6-3=Rz,q4 Ry,q5 Rz,q6
euler angles q4=phi, q5=theta, q6=psi
DH table for 前 3 axies
d1 - height of join1
link    ai  alphai  di  thetai
1       0   90      d1  q1
2       a2  0       0   q2
3       a3  0       0   q3

R3-0=R1-0(q1)R2-1(q2)R3-2(q3)
=[  c1  0   s1
    s1  0   -c1
    0   1   0]
    [c2 -s2 0
    s2  c2  0
    0   0   1]
    [c3 -s3 0
    s3  c3  0
    0   0   1
    ]
    = [c1c23    -c1s23  s1
        s1c23   -s1s23  -c1
        s23     c23     0]
euler angles:
R6-3=R4-3*R5-4*R6-5=[c4c5c6-s4s6    -c4c5s6-s4c6    c4s5
                    s4c5c6+c4s6     -s4c5s6+c4c6    s4s5
                    -s5c6           s5s6            c5]

(R3-0)T * R = [c1c23    s1c23   s23] [r11   r12 r13
                -c1s23  -s1s23  c23   r21   r22 r23
                s1      -c1     0]    r31   r32 r33]
c4s5=c1c23r13+s1c23r23+s23r33
s4s5=-c1s23r13-s1s23r23+c23r33
c5=s1r13-c1r23
1) if s5 !=0, s5=+-Sqrt(1-c5**2)
    q5=atan2(s1r13-c1r23, +-sqrt(1-(s1r13-c1r23)**2)) -> 2 solutions
    i) 取+, s5 > 0
    q4=atan2(c1c23r13+s1c23r23+s23r33, -c1s23r13-s1s23r23+c23r33)
    q6 use R6-3 row3 to cal
    -s5c6=s1r11-c1r23=>s5s6=s1r12-c1r22
    hence, q6=atan2(-s1r11+c1r23,s1r12-c1r22)
ii) s5<0
    q5=atan2(s1r13-c1r23, -sqrt(1-(s1r13-c1r23)**2))
    use R6-3 col 3 to get q4
    q4=atan2(-c4s5, -s4s5)=
    atan2(-c1c23r13-s1c23r23-s23r33, c1s23r13+s1s23r23-c23r33)
    use R6-3 row3 to get q6
    q6=atan2(s1r11-c1r21, -s1r12+c1r22)
2) if s5=0=>q5=0 or pi=>z3,z5 axis on the same line (奇異點)，
we can only know the angle of (q4+q6), we can pick any q4 and then get q6
"""
# return q1, q2, q3
def ik(l1, l2, l3, x, y, phi):
    sol1 = []
    sol2 = []

    r_sq = x**2 + y**2
    # refer to 4-2 example 1
    psi = acos((l2**2 - r_sq - l1**2) / (-2 * l1 * sqrt(r_sq)))
    # solution 1 (q2 > 0 deg)
    sol1.append(atan2(y, x) - psi)
    sol1.append(acos((r_sq - l1**2 - l2**2) / (2 * l1 * l2)))
    sol1.append(phi - sol1[0] - sol1[1])
    #q1_3 = phi - degrees(q1_1) - degrees(q1_2)

    # solution 2 (q2 < 0 deg)
    sol2.append(atan2(y, x) + psi)
    sol2.append(-sol1[1])
    sol2.append(phi - sol2[0] - sol2[1])

    #q3=phi-q1-q2
    # joint space
    #print('sol1:', degrees(q1_1), degrees(q1_2), q1_3)
    #print('sol2:', degrees(q2_1), degrees(q2_2), q2_3)
    print('sol1 (q1~q3):', sol1)
    #print('sol2 (q1~q3):', sol2)
    return sol1

'''
import sympy as sp

# Define symbols
theta1, theta2, l1, l2, x, y = sp.symbols("theta1 theta2 l1 l2 x y")

# Define equations, rearranged so expressions equal 0
eq1 = l1 * sp.cos(theta1) + l2 * sp.cos(theta1 + theta2) - x
eq2 = l1 * sp.sin(theta1) + l2 * sp.sin(theta1 + theta2) - y

# Solve for theta1 & theta2
solution = sp.solve([eq1, eq2], [theta1, theta2], dict=True)
print(solution)
'''