#!/usr/bin/env python

'''
* Team Id : eYRC#1082
* Author List : Rahul,Vinayaka,Khoushikh,Sriram
* Filename: 1082_pos_hold.py
* Theme: Hungry Bird
* Functions: disarm,arm,whycon_callback,altitude_set_pid,pitch_set_pid,roll_set_pid,pid
* Global Variables:
'''
# Importing the required libraries

from plutodrone.msg import *    
from geometry_msgs.msg import PoseArray
from std_msgs.msg import Int16
from std_msgs.msg import Int64
from std_msgs.msg import Float64
from pid_tune.msg import PidTune
import rospy
import time



class Edrone():
	"""docstring for Edrone"""
	
	'''
	* Function Name: __init__
	* Logic: Initialization Function
	'''
	def __init__(self):
		
		rospy.init_node('drone_control')	# initializing ros node with name drone_control

		# This corresponds to your current position of drone. This value must be updated each time in your whycon callback
		# [x,y,z,yaw_value]
		self.drone_position = [0.0,0.0,0.0]	

		# [x_setpoint, y_setpoint, z_setpoint, yaw_value_setpoint]
		self.setpoint = [0.0,-1.0,20.0]


		#Declaring a cmd of message type PlutoMsg and initializing values
		self.cmd = PlutoMsg()
		self.cmd.rcRoll = 1500
		self.cmd.rcPitch = 1500
		self.cmd.rcYaw = 1500
		self.cmd.rcThrottle = 1500
		self.cmd.rcAUX1 = 1500
		self.cmd.rcAUX2 = 1500
		self.cmd.rcAUX3 = 1500
		self.cmd.rcAUX4 = 1500
		# self.cmd.plutoIndex = 0
		self.error = [0.0,0.0,0.0]

		
		self.alt_error = Float64()          #message type for publishing error values
		self.alt_error.data = self.error[2]

		self.pitch_error = Float64()		 #message type for publishing error values
		self.pitch_error.data = self.error[0]

		self.roll_error = Float64()			 #message type for publishing error values
		self.roll_error.data = self.error[1]


		self.zero_line = Float64()			 #message type for publishing error values
		self.zero_line.data = 0.0

		#initial setting of Kp, Kd and ki for [pitch, roll, throttle, yaw]
		self.Kp = [8,8,30,0] 
		self.Ki = [0.01,0.01,0,0]
		self.Kd = [17,17,50,0]

	
		self.prev_errors = [0.0,0.0,0.0]  # to store previous errors

		self.max_values = [1800,1800,1800] # to set maximum and minimum values for pid output.
		self.min_values = [1200,1200,1200]


		########################### SAMPLING TIME#################################################################

		self.sample_time = 0.008 

		###################################################################################################################################


		self.error_sum = [0.0,0.0,0.0] # to store summation of error ( integral of error)

		self.out_pitch = 0  # output values from the pid loop
		self.out_roll = 0
		self.out_throttle = 0
		self.out_yaw = 0
		
		self.proportional = [0,0,0]  # variables required for the pid loop
		self.integral = [0,0,0]
		self.derivative = [0,0,0]


		#----------------------- Publishers------------------------------------------------------------------------------

		# Publishing /drone_command, /alt_error, /pitch_error, /roll_error, /yaw_error to plot it in Plotjuggler
		self.command_pub = rospy.Publisher('/drone_command', PlutoMsg, queue_size=1)

		self.alt_error_pub = rospy.Publisher('/alt_error',Float64, queue_size=1)

		self.pitch_error_pub = rospy.Publisher('/pitch_error',Float64, queue_size=1)

		self.roll_error_pub = rospy.Publisher('/roll_error',Float64, queue_size=1)

		self.zero_line_pub = rospy.Publisher('/zero_line_pub',Float64, queue_size = 1)

		#-------------------------------------------------------------------------------------------------------------------




		#-----------------------------------------------------------------------------------------------------------


		# Subscribing to /whycon/poses, /drone_yaw, /pid_tuning_altitude, /pid_tuning_pitch, pid_tuning_roll
		rospy.Subscriber('whycon/poses', PoseArray, self.whycon_callback)
		rospy.Subscriber('/pid_tuning_altitude',PidTune,self.altitude_set_pid)
		#-------------------------Add other ROS Subscribers here----------------------------------------------------

		rospy.Subscriber('/pid_tuning_pitch',PidTune,self.pitch_set_pid)

		rospy.Subscriber('/pid_tuning_roll',PidTune,self.roll_set_pid)

		#------------------------------------------------------------------------------------------------------------

		self.arm() # ARMING THE DRONE




	'''
	* Function Name:disarm
	* Logic: Disarms the drone
	'''
	def disarm(self):
		self.cmd.rcAUX4 = 1100
		self.command_pub.publish(self.cmd)
		rospy.sleep(1)

	'''
	* Function Name:disarm
	* Logic: Arms the drone
	'''
	def arm(self):

		self.disarm()

		self.cmd.rcRoll = 1500
		self.cmd.rcYaw = 1500
		self.cmd.rcPitch = 1500
		self.cmd.rcThrottle = 1000
		self.cmd.rcAUX4 = 1500
		self.command_pub.publish(self.cmd)	# Publishing /drone_command
		rospy.sleep(1)



	'''
	* Function Name:whycon_callback
	* Input:- whycon coordinates
	* Logic: Callback to update the position of the drone
	'''
	def whycon_callback(self,msg):
		self.drone_position[0] = msg.poses[0].position.y
	
		self.drone_position[1] = msg.poses[0].position.x 

		self.drone_position[2] = msg.poses[0].position.z



	'''
	* Function Name:altitude_set_pid,pitch_set_pid,roll_set_pid
	* Input:- PID constants frpm pidtuner gui
	* Logic: Updates the PID constants
	'''
	def altitude_set_pid(self,alt):
		self.Kp[2] = alt.Kp * 0.04
		self.Ki[2] = alt.Ki * 0.005
		self.Kd[2] = alt.Kd * 0.9
		print(alt)

	def pitch_set_pid(self,pitch):
		self.Kp[0] = pitch.Kp * 0.04 
		self.Ki[0] = pitch.Ki * 0.005
		self.Kd[0] = pitch.Kd * 0.2

	def roll_set_pid(self,roll):
		self.Kp[1] = roll.Kp * 0.06 
		self.Ki[1] = roll.Ki * 0.008
		self.Kd[1] = roll.Kd * 0.3



	#----------------------------------------------------------------------------------------------------------------------
	'''
	* Function Name:pid
	* Input:- Drone position errors
	* Logic: Computes pitch,roll,throttle values to be sent to the drone.
	'''
	def pid(self):
		

####################################### Calculate PID terms #################################################



		self.error = [(self.setpoint[a]-self.drone_position[a]) for a in range(0,3)]  # calculate error array

		self.error_sum = [((self.error_sum[b]+self.error[b])*self.Ki[b]*self.sample_time) for b in range(0,3)] # calculate sum of errors

		self.derivative = [((float(self.error[j]-self.prev_errors[j])/self.sample_time)*self.Kd[j]) for j in range(0,3)] #calculate derivative of errors
 
		self.proportional = [(self.error[n] * self.Kp[n]) for n in range(0,3)] # proportional term in pid equation

		for i in range(len(self.error_sum)):               # limit error_sum to max value
			if self.error_sum[i] > self.max_values[i] :
				self.error_sum[i] = self.max_values[i]

		self.out_pitch = self.proportional[0] + self.error_sum[0] + self.derivative[0]     # pid pitch
		self.out_roll = self.proportional[1] + self.error_sum[1] + self.derivative[1]      # pid roll
		self.out_throttle = self.proportional[2] + self.error_sum[2] + self.derivative[2]  # pid throttle

		self.cmd.rcPitch = 1500 - self.out_pitch			# output Pitch

		self.cmd.rcRoll = 1500 + self.out_roll				# output Roll
		
		self.cmd.rcThrottle = 1500 - self.out_throttle		# output Throttle

####################### Limit output values##############################################################
		if self.cmd.rcPitch > self.max_values[0]:           
			self.cmd.rcPitch = self.max_values[0]
		if self.cmd.rcPitch < self.min_values[0]:
			self.cmd.rcPitch = self.min_values[0]

		if self.cmd.rcRoll > self.max_values[1]:
			self.cmd.rcRoll = self.max_values[1]
		if self.cmd.rcRoll < self.min_values[1]:
			self.cmd.rcRoll = self.min_values[1]

		if self.cmd.rcThrottle > self.max_values[2]:
			self.cmd.rcThrottle = self.max_values[2]
		if self.cmd.rcThrottle < self.min_values[2]:
			self.cmd.rcThrottle = self.min_values[2]
###########################################################################################################


		self.pitch_error.data = self.error[0] # to publish error values
		self.roll_error.data = self.error[1]
		self.alt_error.data = self.error[2]
		

		self.command_pub.publish(self.cmd) # publish the commands

		self.alt_error_pub.publish(self.alt_error) # publish to plotter
		self.pitch_error_pub.publish(self.pitch_error)
		self.roll_error_pub.publish(self.roll_error)
		self.zero_line_pub.publish(self.zero_line)

		
		self.prev_errors = [self.error[m] for m in range(0,3)] # previous error 
		
		rospy.sleep(self.sample_time) 


if __name__ == '__main__':

	e_drone = Edrone()

	while not rospy.is_shutdown():
		e_drone.pid()
		
