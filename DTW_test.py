import pandas as pd
import numpy as np
from math import pi
import time
from dtw import dtw
from scipy.spatial.distance import cityblock

norm_distance = lambda x, y: cityblock(x,y)

def main():

	data1 = np.genfromtxt('/home/zxl5344/test/src/alei/Gaussian_traj/CameraRecording1.csv',delimiter=",",dtype=float)

	data2 = np.genfromtxt('/home/zxl5344/test/src/alei/Gaussian_traj/CameraRecording2.csv',delimiter=",",dtype=float)

	data3 = np.genfromtxt('/home/zxl5344/test/src/alei/Gaussian_traj/CameraRecording3.csv',delimiter=",",dtype=float)
	
	seg1 = np.genfromtxt('/home/zxl5344/test/src/alei/Gaussian_traj/CameraRecording1_seg.csv',delimiter=",",dtype=int)
	seg2 = np.genfromtxt('/home/zxl5344/test/src/alei/Gaussian_traj/CameraRecording2_seg.csv',delimiter=",",dtype=int)
	seg3 = np.genfromtxt('/home/zxl5344/test/src/alei/Gaussian_traj/CameraRecording3_seg.csv',delimiter=",",dtype=int)
	
	x11 = data1[0:seg1[0], 1:4]
	x12 = data1[seg1[1]:seg1[2], 1:4]
	x13 = data1[seg1[2]:seg1[3], 1:4]
	x14 = data1[seg1[3]:seg1[4], 1:4]
	x21 = data2[0:seg2[0], 1:4]
	x22 = data2[seg2[1]:seg2[2], 1:4]
	x23 = data2[seg2[2]:seg2[3], 1:4]
	x31 = data3[0:seg3[0], 1:4]
	x32 = data3[seg3[1]:seg3[2], 1:4]
	x33 = data3[seg3[2]:seg3[3], 1:4]
	
	d11_12 = dtw(x11, x12, dist=norm_distance)
	d11_13 = dtw(x11, x13, dist=norm_distance)
	d11_14 = dtw(x11, x14, dist=norm_distance)
	d11_21 = dtw(x11, x21, dist=norm_distance)
	d11_31 = dtw(x11, x31, dist=norm_distance)
	d11_32 = dtw(x11, x32, dist=norm_distance)
	
	print(d11_12, d11_13, d11_14, d11_21, d11_31, d11_32)


if __name__=='__main__':
    main()
