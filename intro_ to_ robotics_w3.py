'''
Coursera.org 林沛群 Introduction to Robotics 
Week 3
'''

import numpy as np
import math


a0 = 0
a1 = 0
d1 = 0
a2 = 430
d2 = 220
a3 = 0 # z2 z3 垂直
d3 = -90
a4 = 0  # 因为 Z4 Z5 垂直
d4 = 430
a5 = 0
d5 = 0
d6 = 0

theta1=math.radians(15)
theta2=math.radians(-40)
theta3=math.radians(-50)
theta4=math.radians(30)
theta5=math.radians(70)
theta6=math.radians(25)


T01 = np.mat([[math.cos(theta1), -math.sin(theta1), 0, a0], 
    [math.sin(theta1), math.cos(theta1), 0, 0],
    [0, 0, 1, d1], 
    [0, 0, 0, 1]])

T12 =  np.mat([[math.cos(theta2), -math.sin(theta2), 0, a1], 
    [0, 0, 1, d2], 
    [-math.sin(theta2), -math.cos(theta2), 0, 0], 
    [0, 0, 0, 1]])

T23 =  np.mat([[math.cos(theta3), -math.sin(theta3), 0, a2], 
    [math.sin(theta3), math.cos(theta3), 0, 0], 
    [0, 0, 1, d3], 
    [0, 0, 0, 1]])

T34 =  np.mat([[math.cos(theta4), -math.sin(theta4), 0, a3], 
    [0, 0, 1, d4], 
    [-math.sin(theta4), -math.cos(theta4), 0, 0], 
    [0, 0, 0, 1]])

T45 =  np.mat([[math.cos(theta5), -math.sin(theta5), 0, a4], 
    [0, 0, -1, -d5], 
    [math.sin(theta5), math.cos(theta5), 0, 0], 
    [0, 0, 0, 1]])

T56 =  np.mat([[math.cos(theta6), -math.sin(theta6), 0, a5], 
    [0, 0, 1, d6], 
    [-math.sin(theta6), -math.cos(theta6), 0, 0], 
    [0, 0, 0, 1]])

# NO.4 
P0 = np.mat([[0],[0],[0],[1]])
print("NO.4\n")
print(T01 * T12 * T23 * T34 * P0)

# NO.5
print("NO.5\n")
print(T01 * T12 * T23 * T34 * T45 * T56)
