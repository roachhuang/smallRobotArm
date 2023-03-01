import numpy as np
name = ["Manjeet", "Nikhil", "Shambhavi", "Astha"]
roll_no = [4, 1]+[2,5]

# using zip() to map values
mapped = zip(roll_no, name)
x = tuple(mapped)
print(type(x))
for i in x:
    print(i)

x= np.array([[-1.3231, -0.5],[0.5, -1.3231]])
y=np.array([1,1])
z= np.linalg.inv(x) @ y

print(np.rad2deg(np.arctan2(-0.9113, -0.4114)))

print(np.append([[1, 2, 3], [4, 5, 6]], [[7, 8, 9]], axis=0))