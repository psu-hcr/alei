import numpy as np
from numpy import genfromtxt
import pandas as pd

class lpf:
	"""
	This is a class as offline low pass filter
	"""
	
	def __init__(self, _data, _path):
		self.data = _data
		self.path = _path
		
	def filter(self, x_curr, x_prev, y_prev):
		return (x_curr+x_prev+0.376*y_prev)/2.376
	
	def calc(self):
		self.newdata = np.copy(self.data)
		for i in range(1, len(self.data)):
			x_curr = self.data[i].T
			x_prev = self.data[i-1].T
			y_prev = self.newdata[i-1].T
			y_curr = self.filter(x_curr, x_prev, y_prev)
			self.newdata[i] = y_curr.T
	
	def save(self):
		df = pd.DataFrame(self.newdata)
		df.to_csv(self.path, header=False, index=False)
	
def main():
	data1 = genfromtxt('/home/zxl5344/test/src/alei/Gaussian_traj/CameraRecording_new1.csv',delimiter=",",dtype=float)
	data1 = np.delete(data1,0,0)
	data1 = data1[:,:4]
	print(data1.shape)
	path = '/home/zxl5344/test/src/alei/Gaussian_traj/CameraRecording_new1_f.csv'
	LPF = lpf(data1,path)
	LPF.calc()
	LPF.save()
	
if __name__=='__main__':
    main()

