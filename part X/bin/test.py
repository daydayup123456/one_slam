import numpy as np
import matplotlib.pyplot as plt
import mpl_toolkits.mplot3d
import time
f1 = open("./groundtruth1.txt")
x1 = []
y1 = []
z1 = []
for line in f1:
    if line[0] == '#':
        continue
    data = line.split()
    x1.append( float(data[1] ) )
    y1.append( float(data[2] ) )
    z1.append( float(data[3] ) )
ax = plt.subplot( 111, projection='3d')
ax.plot(x1,y1,z1,"b")

f2 = open("./groundtruth2.txt")
x2 = []
y2 = []
z2 = []
for line in f2:
    if line[0] == '#':
        continue
    data = line.split()
    x2.append( float(data[2] ) )
    y2.append( float(data[3] ) )
    z2.append( float(data[4] ) )
ax.plot(x2,y2,z2,"r")

plt.show()

