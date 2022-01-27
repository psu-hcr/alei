#!/usr/bin/env python
"""
Zhiqing Lu

This node move robot to a desire pose in world frame. It use iiwa_ik_server to solve ik and use /iiwa/iiwa_fk_server to provide current pose.
SUBSCRIBERS:
	- /iiwa/joint_states (JointState)
PUBLISHERS:
	- /iiwa/PositionController/command (Float64MultiArray)
SERVICES:
	- /iiwa/iiwa_ik_server
	- /iiwa/iiwa_fk_server

"""

import rospy
from std_msgs.msg import Float64MultiArray, MultiArrayLayout, MultiArrayDimension
from sensor_msgs.msg import JointState
from iiwa_tools.srv import GetIK, GetIKRequest, GetFK
from geometry_msgs.msg import Pose
import pandas as pd
import numpy as np


class Move:
	"""
	this class let robot move to a pose in world frame. 
	"""
	
	def __init__(self, _tarj):
		rospy.loginfo("Robot starts moving")
		
		# time duration for publishing joint state
		self.DT = 1./500.
		
		# time duration for torque calculation
		self.DT_calc = 0.05
		
		# setup state variables
		self.rob_state = JointState()
		
		# initial storing
		self.measurement = np.zeros(14)
		
		# initialize traj array 
		self.traj = _tarj[:, 32:39]
		print(self.traj.shape)
		
		# Total step
		self.total_step = len(self.traj)
		
		# initialize timer and record begin time
		self.t = 0
		self.begintime = rospy.get_rostime()
		
		# setup and initialize goal
		self.goal = Float64MultiArray()
		self.goal.data = [0, 0, 0, 0, 0, 0, 0]
		
		# setup a step counter
		self.step = 0
		
		# setup shutdown function
		rospy.on_shutdown(self.back2zeros)
		
		# setup FK client
		self.fk_service = '/iiwa/iiwa_fk_server'
		rospy.wait_for_service(self.fk_service)
		self.get_fk = rospy.ServiceProxy(self.fk_service, GetFK)
		self.seed = Float64MultiArray()
		self.seed.layout = MultiArrayLayout()
		self.seed.layout.dim = [MultiArrayDimension(), MultiArrayDimension()]
		self.seed.layout.dim[0].size = 1
		self.seed.layout.dim[1].size = 7
		self.seed.data = self.rob_state.position
		
		# setup publishers, subscribers, timers:
		self.move_pub=rospy.Publisher("/iiwa/PositionController/command", Float64MultiArray, queue_size=10)
		self.joint_sub=rospy.Subscriber("/iiwa/joint_states", JointState, self.update_state, queue_size=10)
		self.sim_timer = rospy.Timer(rospy.Duration(self.DT),self.moving)
		self.calc_timer = rospy.Timer(rospy.Duration(self.DT_calc),self.update_goal)
		
		
	def moving(self,data):
		"""
		publishing current desire joint state
		"""
		self.move_pub.publish(self.goal)
		
	def update_goal(self,data):
		"""
		update desire joint state
		"""
		if self.step<self.total_step:
			self.goal.data = self.traj[self.step]
			self.step = self.step + 1
		
	def update_state(self,data):
		"""
		update data and storing position, velocity effort, end effector pos & orientation
		"""
		self.rob_state = data
		self.measurement = np.vstack((self.measurement, np.concatenate((self.rob_state.position,self.rob_state.velocity), axis=None)))
	
	def currentposition(self):
		"""
		print current pose
		"""
		self.seed.data = self.rob_state.position
		self.respFK = self.get_fk(self.seed)
		self.sol_pose = self.respFK.poses[0]
		
	def back2zeros(self):
		"""
		move robot to zero position
		"""
		self.calc_timer.shutdown()  # stop the calc_timer
		rospy.loginfo("Move to zero pose ")
		self.goal.data = [0, 0, 0, 0, 0, 0, 0]
		rospy.sleep(5)
		
def main():
	# convert data from datasheets into arrays
	df = pd.read_csv("/home/zxl5344/test/src/alei/robotdata/f8IK.csv")
	data = pd.DataFrame.to_numpy(df)
	data = np.delete(data,0,0)
	
	rospy.init_node('Start', log_level=rospy.INFO)

	try:
		test = Move(data)
		rospy.spin()   
		  
	except rospy.ROSInterruptException: 
		pass
	
	rospy.loginfo("Task complete")
	
	
	# output data to excel
	df = pd.DataFrame(test.measurement)
	path = '/home/zxl5344/test/src/alei/robotdata/trajPosControl.csv'
	df.to_csv(path, header=False, index=False)


if __name__=='__main__':
    main()
