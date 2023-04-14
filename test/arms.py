import math
import matplotlib.pyplot as plt

l1 = 0.8
l2 = 0.5

n_theta = 10  # No of divisions
theta_start = 0  # Starting angle
theta_end = math.pi / 2  # Ending angle
theta1 = []
theta2 = []

for i in range(0, n_theta):
    theta1.append(theta_start + (theta_end - theta_start) * i /
                  (n_theta - 1))  # Angles of link 1
    theta2.append(theta_start + (theta_end - theta_start) * i /
                  (n_theta - 1))  # Angles of link 2

# Base posotion
x0 = 0  # Base position
y0 = 0  # Base position
ct = 1  # Counter

# Link 1 end point
for t1 in theta1:
    x1 = l1 * math.cos(t1)  # End of link 1
    y1 = l1 * math.sin(t1)  # End of link 1
    for t2 in theta2:

        x2 = x1 + l2 * math.cos(t2)  # End of link 2
        y2 = y1 + l2 * math.sin(t2)  # End of link 2

        filename = str(ct) + '.png'
        ct = ct + 1
        #plt.figure()
        plt.plot([x0, x1], [y0, y1])
        plt.plot([x1, x2], [y1, y2])
        plt.xlim([-1, 2])
        plt.ylim([-1, 2])
        plt.pause(0.2)
        # plt.savefig(filename)
plt.show()
