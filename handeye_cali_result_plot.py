# this function plot the result from handeye calibration evaluator

import numpy as np
import pandas as pd
from matplotlib import pyplot as plt

def main():
	# convert data from datasheets into arrays
	df = pd.read_csv("/home/zxl5344/IROS2022/src/iiwa_ros/iiwa_ros/lus-rosscripts/calibration_evalu.csv")
	measuredata = pd.DataFrame.to_numpy(df)
	measuredata = np.delete(measuredata, (0), axis=0)
	
	# Get total number of data
	datalength = (len(measuredata[:,0]))
	
	# plot real data and Koopman guess data
	t = np.arange(datalength)
	
	# translation
	fig0 = plt.figure(1)
	fig0.suptitle('Translation', fontsize=16)
	plt.plot(t, measuredata[:, 0], label='x')
	plt.plot(t, measuredata[:, 1], label='y')
	plt.plot(t, measuredata[:, 2], label='z')
	plt.legend()
	
	# Rotation
	fig1 = plt.figure(2)
	fig1.suptitle('Rotation', fontsize=16)
	plt.plot(t, measuredata[:, 3], label='x')
	plt.plot(t, measuredata[:, 4], label='y')
	plt.plot(t, measuredata[:, 5], label='z')
	plt.plot(t, measuredata[:, 6], label='w')
	plt.legend()
	
	plt.show()

if __name__=='__main__':
    main()