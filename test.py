from cmath import acos, pi
from math import atan2
import numpy as np

a1 = -30
print((227)**2 + (372)**2 + 188.6**2 + 2 * (227 + 372) * 30 + 30**2)
print((435.79 - a1)**2 + 188.6**2)
c = (252530 - 40**2 - 338**2 - 340**2) / (2 * 340)
print('c', c)
a = -40
b = 338
#a*cos-b*sin=c
r = np.sqrt(a**2 + b**2)
alp = atan2(b, a)
#r*cos(q+alp)=c
# or 360-
qNalp1 = acos(c / r)
qNalp2 = 2 * pi - qNalp1
q1 = qNalp1 - alp
q2 = qNalp2 - alp
print(q1 * 180 / pi, q2 * 180 / pi)

import numpy as np
a1 = ['aaa', 'bbb', 'ccc']
a2 = ['ppp', 'qqq', 'rrr']
print("\na1 : ", a1)
print("\na2 : ", a2)
print("\na1 : ", np.char.multiply(a1, a2))
print ("\na1 : ", np.char.multiply(a1, [2, 4, 3]))
print ("\na2 : ", np.char.multiply(a2, 3))

def dh():
    a = np.array([['c2', '-s2', '0', 'a1'], ['0', '0', '1', '0'],
                     ['-s2', '-c2', '0', '0'], ['0', '0', '0', '1']], dtype='S')

    b = np.array([['c3', '-s3', '0', 'a2'], ['s3', 'c3', '0', '0'],
                     ['0','0', '1', '0'], ['0', '0', '0', '1']], dtype='S')

    a@b
    #[y + z for (y, z) in zip(a, [str(i) for i in b])]


#dh()