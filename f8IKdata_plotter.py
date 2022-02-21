import numpy as np
from numpy import genfromtxt
import matplotlib.pyplot as plt

data=genfromtxt('/home/zxl5344/test/src/alei/robotdata/f8IK.csv',delimiter=",",dtype=float)
data = np.delete(data,0,0)
time = data[:,0]
poserr1 = data[:, 1]
poserr2 = data[:, 2]
poserr3 = data[:, 3]
poserr4 = data[:, 4]
poserr5 = data[:, 5]
poserr6 = data[:, 6]
poserr7 = data[:, 7]
vel1 = data[:, 8]
vel2 = data[:, 9]
vel3 = data[:, 10]
vel4 = data[:, 11]
vel5 = data[:, 12]
vel6 = data[:, 13]
vel7 = data[:, 14]
u1 = data[:, 15]
u2 = data[:, 16]
u3 = data[:, 17]
u4 = data[:, 18]
u5 = data[:, 19]
u6 = data[:, 20]
u7 = data[:, 21]
pos1 = data[:, 25]
pos2 = data[:, 26]
pos3 = data[:, 27]
pos4 = data[:, 28]
pos5 = data[:, 29]
pos6 = data[:, 30]
pos7 = data[:, 31]
djs1 = data[:, 32]
djs2 = data[:, 33]
djs3 = data[:, 34]
djs4 = data[:, 35]
djs5 = data[:, 36]
djs6 = data[:, 37]
djs7 = data[:, 38]
fisher = data[:, 39]

pos1_max = max(pos1.min(), pos1.max(), key=abs)
pos2_max = max(pos2.min(), pos2.max(), key=abs)
pos3_max = max(pos3.min(), pos3.max(), key=abs)
pos4_max = max(pos4.min(), pos4.max(), key=abs)
pos5_max = max(pos5.min(), pos5.max(), key=abs)
pos6_max = max(pos6.min(), pos6.max(), key=abs)
pos7_max = max(pos7.min(), pos7.max(), key=abs)
print("Max of joint")
print(pos1_max, pos2_max, pos3_max, pos4_max, pos5_max, pos6_max, pos7_max)


# error plot
plt.figure(1)
plt.title("Joint 1 error")
plt.plot(time,poserr1,'r')

plt.figure(2)
plt.title("Joint 2 error")
plt.plot(time,poserr2,'r')

plt.figure(3)
plt.title("Joint 3 error")
plt.plot(time,poserr3,'r')

plt.figure(4)
plt.title("Joint 4 error")
plt.plot(time,poserr4,'r')

plt.figure(5)
plt.title("Joint 5 error")
plt.plot(time,poserr5,'r')

plt.figure(6)
plt.title("Joint 6 error")
plt.plot(time,poserr6,'r')

plt.figure(7)
plt.title("Joint 7 error")
plt.plot(time,poserr7,'r')

"""
# desire joint state plot
plt.figure(8)
plt.title("desire Joint 1")
plt.plot(time,djs1,'g^')

plt.figure(9)
plt.title("desire Joint 2")
plt.plot(time,djs2,'g^')

plt.figure(10)
plt.title("desire Joint 3")
plt.plot(time,djs3,'g^')

plt.figure(11)
plt.title("desire Joint 4")
plt.plot(time,djs4,'g^')

plt.figure(12)
plt.title("desire Joint 5")
plt.plot(time,djs5,'g^')

plt.figure(13)
plt.title("desire Joint 6")
plt.plot(time,djs6,'g^')

plt.figure(14)
plt.title("desire Joint 7")
plt.plot(time,djs7,'g^')
"""
"""
# fisher cost plot
plt.figure(15)
plt.title("current fisher cost")
plt.plot(time,fisher,'k')
"""
"""
# velocity plot
plt.figure(16)
plt.title("vel 1")
plt.plot(time,vel1,'b')

plt.figure(17)
plt.title("vel 2")
plt.plot(time,vel2,'b')

plt.figure(18)
plt.title("vel 3")
plt.plot(time,vel3,'b')

plt.figure(19)
plt.title("vel 4")
plt.plot(time,vel4,'b')

plt.figure(20)
plt.title("vel 5")
plt.plot(time,vel5,'b')

plt.figure(21)
plt.title("vel 6")
plt.plot(time,vel6,'b')

plt.figure(22)
plt.title("vel 7")
plt.plot(time,vel7,'b')
"""

# input plot
plt.figure(23)
plt.title("u1")
plt.plot(time,u1,'m')

plt.figure(24)
plt.title("u2")
plt.plot(time,u2,'m')

plt.figure(25)
plt.title("u3")
plt.plot(time,u3,'m')

plt.figure(26)
plt.title("u4")
plt.plot(time,u4,'m')

plt.figure(27)
plt.title("u5")
plt.plot(time,u5,'m')

plt.figure(28)
plt.title("u6")
plt.plot(time,u6,'m')

plt.figure(29)
plt.title("u7")
plt.plot(time,u7,'m')

# pose plot
plt.figure(30)
plt.title("pos1")
plt.plot(time,pos1,'y')

plt.figure(31)
plt.title("pos2")
plt.plot(time,pos2,'y')

plt.figure(32)
plt.title("pos3")
plt.plot(time,pos3,'y')

plt.figure(33)
plt.title("pos4")
plt.plot(time,pos4,'y')

plt.figure(34)
plt.title("pos5")
plt.plot(time,pos5,'y')

plt.figure(35)
plt.title("pos6")
plt.plot(time,pos6,'y')

plt.figure(36)
plt.title("pos7")
plt.plot(time,pos7,'y')

plt.show()