import numpy as np
import matplotlib.pyplot as plt

x = np.linspace( 0,10 )

for n in range(3):
    y = np.sin( x+n )
    #plt.figure()
    plt.plot( x, y )
    plt.pause(0.05)
plt.show()



coordinates = [
    (524.447876,1399.091919),
    (525.1377563,1399.95105),
    (525.7932739,1400.767578),
    (526.4627686,1401.601563),
    (527.2360229,1402.564575),
    (527.8989258,1403.390381),
    (528.5689697,1404.224854),
]

timestamp = [0, 0.05, 0.1, 0.15, 0.2, 0.25, 0.3]

x, y = zip(*coordinates)

ax = plt.axes(projection="3d")
ax.plot(x, y, timestamp);

plt.show()
# Create Scatter Plot

#3d
ax = plt.axes(projection='3d')

# Data for a three-dimensional line
zline = np.linspace(0, 15, 1000)
xline = np.sin(zline)
yline = np.cos(zline)
ax.plot3D(xline, yline, zline, 'gray')

# Data for three-dimensional scattered points
zdata = 15 * np.random.random(100)
xdata = np.sin(zdata) + 0.1 * np.random.randn(100)
ydata = np.cos(zdata) + 0.1 * np.random.randn(100)
ax.scatter3D(xdata, ydata, zdata, c=zdata, cmap='Greens')
plt.show()
'''
x=0
for i in range(10):
    x=np.sin(i)
    y = np.cos(30*x-5)

    plt.title("Scatter Plot")
    plt.xlabel("X-Axis")
    plt.ylabel("Y-Axis")

    plt.plot(x, y)
    plt.figure()
    plt.pause(0.2)

# Display

plt.show()
'''