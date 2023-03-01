from math import radians
import numpy as np
import craig as cg

np.set_printoptions(precision=4, suppress=True)
def q6():
    ap=np.array([0,3,4])
    rotab=np.array([[0, 0.707, -0.707],[0, 0.707, 0.707], [1,0,0]])
    print (ap)
    print (rotab)
    # order matters
    print ('a6:', rotab.T @ ap.T)

def q7():
    # np.set_printoptions(precision=4, suppress=True)
    ap=np.array([2, 0, 4])
    ry=cg.Rot('y', radians(90))

    print('a7:', ry)
q6()
q7()

