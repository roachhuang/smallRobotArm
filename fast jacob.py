import roboticstoolbox as rtb
import numpy as np
import spatialmath as sm
import qpsolvers as qp
import math
import swift
import spatialgeometry as sg

def step_robot(r, Tep):
    env=swift.Swift()
    env.launch(realtime=True)
    

rb = rtb.models.Panda()

J_slow = rb.jacobe(rb.qr)
J_fast = rb.jacobe(rb.qr, fast=True)

print(np.round(J_slow,2))
print(np.round(J_fast,2))
