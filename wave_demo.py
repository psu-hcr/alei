#!/usr/bin/env python
"""
Zhiqing Lu

This node follows a desire trajectory.
SUBSCRIBERS:
	- /iiwa/joint_states (JointState)
PUBLISHERS:
	- /iiwa/PositionController/command (Float64MultiArray)
SERVICES:

"""

import rospy
from std_msgs.msg import Float64MultiArray, MultiArrayLayout, MultiArrayDimension
from sensor_msgs.msg import JointState
from iiwa_tools.srv import GetIK, GetIKRequest, GetFK
from geometry_msgs.msg import Pose
from math import cos, pi
import numpy as np



class Waver:
	
	def __init__(self):
		rospy.loginfo("Robot starts moving")
		self.DT=1/500							# change frequence to 500hz
		
		# setup state variables
		self.rob_state=JointState()
		
		# initial storing
		self.measurement = np.zeros(28)
		
		# initial time
		self.starttime = rospy.get_rostime().secs
		
		# setup Torque
		self.Torque = Float64MultiArray()
		self.Torque.data = [0, 0, 0, 0, 0, 0, 0]
		
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
		self.move_pub=rospy.Publisher("/iiwa/TorqueController/command", Float64MultiArray, queue_size=10)
		self.joint_sub=rospy.Subscriber("/iiwa/joint_states", JointState, self.update_state, queue_size=10)
		rospy.wait_for_message("/iiwa/joint_states", JointState)
		self.sim_timer = rospy.Timer(rospy.Duration(self.DT),self.moving)
		
		# setup goal joint state
		self.goal = Float64MultiArray()
		self.goal.data = [0, 0, 0, 0, 0, 0, 0]
		
		
		# setup constant applied Torque
		self.constantT = [20, 90, 30, 30, 5, 4, 0.5]
		
		# setup shutdown
		rospy.on_shutdown(self.back2zeros)
		
	def moving(self,data):
		"""
		Publish Torque
		"""
		self.CaluTorque()
		self.move_pub.publish(self.Torque)
		
		
	def update_state(self,data):
		"""
		update data and storing position, velocity ,effort, end effector pos and orientation
		"""
		self.rob_state = data
		self.currentpose()
		self.measurement = np.vstack((self.measurement, np.concatenate((self.sol_pose.position.x,self.sol_pose.position.y, self.sol_pose.position.z, self.sol_pose.orientation.x, self.sol_pose.orientation.y, self.sol_pose.orientation.z, self.sol_pose.orientation.w,self.rob_state.position,self.rob_state.velocity,self.Torque.data), axis=None)))
		
	def back2zeros(self):
		"""
		Move robot back to zeros pos and shut down
		"""
		rospy.loginfo("closing node")
		self.goal.data = [0, 0, 0, 0, 0, 0, 0]
		rospy.sleep(5.)
		
		
	def CaluTorque(self):
		"""
		calculate sinusoidal Torque
		"""
		# current time
		now = rospy.get_rostime().secs - self.starttime
		#print('now:')
		#print(now)
		
		# calculate cos
		cosine = cos(pi*now)
		
		# calculate force
		
		Force = [0, 0, 0, 0, 0, 0, 0]
		
		Force[0] = cosine * self.constantT[0]
		Force[1] = cosine * self.constantT[1]
		Force[2] = cosine * self.constantT[2]
		Force[3] = cosine * self.constantT[3]
		Force[4] = cosine * self.constantT[4]
		Force[5] = cosine * self.constantT[5]
		Force[6] = cosine * self.constantT[6]
		
		
		self.Torque.data = Force
		
	def currentpose(self):
		"""
		print current pose
		"""
		self.seed.data = self.rob_state.position
		self.respFK = self.get_fk(self.seed)
		self.sol_pose = self.respFK.poses[0]
		#print('current pose:')
		#print(self.sol_pose)
		
	def currentposition(self):
		"""
		print out current position
		"""
		
		print('current joint:')
		print(self.rob_state.position)
		
		

def main():
    """
    Run the main loop, by instatiating a waver class, and then calling ros.spin
    """
    rospy.init_node('wave', log_level=rospy.INFO)

    try:
        test = Waver()
    except rospy.ROSInterruptException: 
    	pass

    rospy.spin()


if __name__=='__main__':
    main()
