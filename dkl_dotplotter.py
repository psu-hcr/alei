import numpy as np
from numpy import genfromtxt
import matplotlib.pyplot as plt

data=genfromtxt('/home/zxl5344/test/src/alei/robotdata/dklsac_dot.csv',delimiter=",",dtype=float)
data = np.delete(data,0,0)
time = data[:,0]
pos1 = data[:, 1]
pos2 = data[:, 2]
pos3 = data[:, 3]
wx = data[:, 4]
wy = data[:, 5]
wz = data[:, 6]
w = data[:, 7]


# plot
plt.figure(1)
plt.title("theta 1, 2, 3")
plt.plot(time,pos1,'r', label='theta1')
plt.plot(time,pos2,'g', label='theta2')
plt.plot(time,pos3,'b', label='theta3')
plt.legend()

plt.show()
