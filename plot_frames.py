import spatialmath.base as smb
import matplotlib.pyplot as plt
from spatialmath import SE3

# Define the transformation matrices for each frame
T0 = smb.trnorm(smb.transl(0, 0, 0))
T1 = smb.trnorm(smb.transl(1, 0, 0))
T2 = smb.trnorm(smb.transl(1, 1, 0))
T3 = smb.trnorm(smb.transl(1, 1, 1))

# Plot the frames
# fig = plt.figure()
# ax = fig.add_subplot(111, projection='3d')


# smb.trplot(T0, frame='0', ax=ax)
# smb.trplot(T1, frame='1', ax=ax)
# smb.trplot(T2, frame='2', ax=ax)
# smb.trplot(T3, frame='3', ax=ax)

# plt.show()

import matplotlib.pyplot as plt
import spatialmath.base as smb
from spatialmath import SE3

fig = plt.figure()
ax = fig.add_subplot(111, projection="3d")

# Define transformations
# T1 = SE3.Trans(1, 2, 3)  # Translation only
# T2 = SE3.Trans(2, 3, 4)  # Another translation

# Plot frames
smb.trplot(T1, frame="T1", ax=ax, length=0.2)
smb.trplot(T2, frame="T2", ax=ax, length=0.2)

# plt.show()


import matplotlib.pyplot as plt
from spatialmath.base import trplot, transl, rpy2tr
fig = plt.figure(figsize=(10,10))
text_opts = dict(bbox=dict(boxstyle="round", fc="w", alpha=0.9), zorder=20, family='monospace', fontsize=8, verticalalignment='top') 
T = transl(2, 1, 1)@ rpy2tr(0, 0, 0)
ax = fig.add_subplot(331, projection='3d')
trplot(T, ax=ax, dims=[0,4])
ax.text(0.5, 0.5, 4.5, "trplot(T)", **text_opts)
ax = fig.add_subplot(332, projection='3d')
trplot(T, ax=ax, dims=[0,4], originsize=0)
ax.text(0.5, 0.5, 4.5, "trplot(T, originsize=0)", **text_opts) 
ax = fig.add_subplot(333, projection='3d')
trplot(T, ax=ax, dims=[0,4], style='line')
ax.text(0.5, 0.5, 4.5, "trplot(T, style='line')", **text_opts)
ax = fig.add_subplot(334, projection='3d')
trplot(T, ax=ax, dims=[0,4], axislabel=False)
ax.text(0.5, 0.5, 4.5, "trplot(T, axislabel=False)", **text_opts)
ax = fig.add_subplot(335, projection='3d')
trplot(T, ax=ax, dims=[0,4], width=3, color=('r', 'g', 'b'))
ax.text(0.5, 0.5, 4.5, "trplot(T, width=3)", **text_opts)
ax = fig.add_subplot(336, projection='3d')
trplot(T, ax=ax, dims=[0,4], frame='B')
ax.text(0.5, 0.5, 4.5, "trplot(T, frame='B')", **text_opts)
ax = fig.add_subplot(337, projection='3d')
trplot(T, ax=ax, dims=[0,4], color='r', textcolor='k')
ax.text(0.5, 0.5, 4.5, "trplot(T, color='r', textcolor='k')", **text_opts)
ax = fig.add_subplot(338, projection='3d')
trplot(T, ax=ax, dims=[0,4], labels=("u", "v", "w"))
ax.text(0.5, 0.5, 4.5, "trplot(T, labels=('u', 'v', 'w'))", **text_opts) 
ax = fig.add_subplot(339, projection='3d')
trplot(T, ax=ax, dims=[0,4], style='rviz')
ax.text(0.5, 0.5, 4.5, "trplot(T, style='rviz')", **text_opts)

plt.show()