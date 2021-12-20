import numpy as np
from numpy import genfromtxt
import matplotlib.pyplot as plt

data=genfromtxt('/home/zxl5344/test/src/alei/CameraRecording.csv',delimiter=",",dtype=float)
data = np.delete(data,0,0)
data = np.delete(data,0,0)
time = data[0:-1,0]
pegX = data[0:-1,1]
pegY = data[0:-1,2]
pegZ = data[0:-1,3]
baseX = data[0:-1,4]
baseY = data[0:-1,5]
baseZ = data[0:-1,6]

plt.figure(1)
plt.title("peg X")
plt.plot(time,pegX,'g')

plt.figure(2)
plt.title("peg Y")
plt.plot(time,pegY,'r')

plt.figure(3)
plt.title("peg Z")
plt.plot(time,pegZ,'k')


plt.figure(4)
plt.title("baseX")
plt.plot(time,baseX,'g')

plt.figure(5)
plt.title("baseY")
plt.plot(time,baseY,'r')

plt.figure(6)
plt.title("baseZ")
plt.plot(time,baseZ,'k')

plt.show()
