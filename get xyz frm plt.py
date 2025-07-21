import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np


def rose_xy_position(t, R=40):
    return R * np.cos(3*t) * np.cos(t), R * np.cos(3*t) * np.sin(t)

ts = np.linspace(0, 2*np.pi, 500)
xs, ys = zip(*[rose_xy_position(t) for t in ts])
plt.plot(xs, ys); plt.axis("equal")

plt.show()

input("Press Enter to continue...")

# Generate some random data
x = np.random.rand(100)
y = np.random.rand(100)
z = x**2 + y**2

# Create a scatter plot of the data
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
scatter = ax.scatter(x, y, c=z)

def get_coord(x,y):
    # store the current mousebutton
    b = ax.button_pressed
    # set current mousebutton to something unreasonable
    ax.button_pressed = -1
    # get the coordinate string out
    s = ax.format_coord(x,y)
    # set the mousebutton back to its previous state
    ax.button_pressed = b
    return s

def on_move(event):   
    if event.inaxes == ax:
        str_xyz =get_coord(event.xdata, event.ydata).rstrip()
        list_xyz =str_xyz.split(',')
        num_xyz = [float(value) for key, value in [pair.split('=') for pair in list_xyz]]         
        print(num_xyz)
    
    # print(f"x: {x:.2f}, y: {y:.2f}, z: {z:.2f}")
   
#fig.canvas.mpl_connect('motion_notify_event', on_move)
fig.canvas.callbacks.connect('button_press_event', on_move)
plt.show()
