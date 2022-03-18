import pandas as pd
import numpy as np
from math import pi
import time
from dtw import dtw

def main():

	data1 = np.genfromtxt('/home/zxl5344/test/src/alei/Gaussian_traj/CameraRecording1.csv',delimiter=",",dtype=float)
	x1 = data1[:, 1]
	y1 = data1[:, 2]
	z1 = data1[:, 3]

	data2 = np.genfromtxt('/home/zxl5344/test/src/alei/Gaussian_traj/CameraRecording2.csv',delimiter=",",dtype=float)
	x2 = data2[:, 1]
	y2 = data2[:, 2]
	z2 = data2[:, 3]

	data3 = np.genfromtxt('/home/zxl5344/test/src/alei/Gaussian_traj/CameraRecording3.csv',delimiter=",",dtype=float)
	x3 = data3[:, 1]
	y3 = data3[:, 2]
	z3 = data3[:, 3]
	
	seg1 = np.genfromtxt('/home/zxl5344/test/src/alei/Gaussian_traj/CameraRecording1_seg.csv',delimiter=",",dtype=float)
	seg2 = np.genfromtxt('/home/zxl5344/test/src/alei/Gaussian_traj/CameraRecording2_seg.csv',delimiter=",",dtype=float)
	seg3 = np.genfromtxt('/home/zxl5344/test/src/alei/Gaussian_traj/CameraRecording3_seg.csv',delimiter=",",dtype=float)


if __name__=='__main__':
    main()
