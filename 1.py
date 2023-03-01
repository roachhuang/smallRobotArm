# Import math Library
import math
import numpy as np
import pandas as pd

from sympy import Identity, Matrix, MatrixSymbol, Symbol, init_printing, symbols

init_printing(use_unicode=True)

m = Matrix([1, 2, 3])
#print(m)

q1 = Symbol('q1')
q2 = Symbol('q2')
q3 = Symbol('q3')

X = MatrixSymbol('X', 3, 3)
Y = MatrixSymbol('Y', 3, 3)
(X.T * X).I * Y
print(X**(-1) * X.T**(-1) * Y)
print(Matrix(X))

# Return the sine value of 30 degrees
#print(round(math.cos(math.radians(-90))))


column_names = ['a', 'b', 'c']
row_names    = ['1', '2', '3']

matrix = np.reshape((10, 20, 30, 40, 50, 60, 70, 80, 90), (3, 3))
df = pd.DataFrame(matrix, columns=column_names, index=row_names)
print(df)

import matplotlib.pyplot as plt
import numpy as np
import time


fig = plt.figure()
ax = fig.add_subplot(projection='3d')

# Make the X, Y meshgrid.
xs = np.linspace(-1, 1, 50)
ys = np.linspace(-1, 1, 50)
X, Y = np.meshgrid(xs, ys)

# Set the z axis limits so they aren't recalculated each frame.
ax.set_zlim(-1, 1)

# Begin plotting.
wframe = None
tstart = time.time()
for phi in np.linspace(0, 180. / np.pi, 100):
    # If a line collection is already remove it before drawing.
    if wframe:
        wframe.remove()
    # Generate data.
    Z = np.cos(2 * np.pi * X + phi) * (1 - np.hypot(X, Y))
    # Plot the new wireframe and pause briefly before continuing.
    wframe = ax.plot_wireframe(X, Y, Z, rstride=2, cstride=2)
    plt.pause(.001)

print('Average FPS: %f' % (100 / (time.time() - tstart)))