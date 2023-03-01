# cartesion space
import numpy as np
# import pandas as pd
# import matplotlib.pyplot as plt
import plan_traj as pt

np.set_printoptions(precision=2, suppress=True)

# col_names = ['ti', 'xi', 'yi', 'qi']
# 4 points has 3 linear lines and 4個2次段
p = np.array([[0, -4, 0, 120], [2, -5, 5, 45], [4, 2, 3, 30], [9, 5, -3, 0]])

pt.planTraj(p)

'''
for t in 9s: every 0.25s
    switch(t):
        case (t in 0, 0.5)
            set a0 to motor
        case (t in 0.5, 1.75)
            set v1 to motor
        case (t in 1.75, 2.25)
            set a1 to motor
        case (t in 2.25, 3.75)
            set v2 to motor

import time
def setInterval(func, sec):
    time.sleep(sec)
    func()
    setInterval(func(), sec)

def call():
    print('hello!')

setInterval(call, 3)
'''