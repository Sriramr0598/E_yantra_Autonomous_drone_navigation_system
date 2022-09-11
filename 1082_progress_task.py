#!/usr/bin/env python

'''

This python file runs a ROS-node of name drone_control which holds the position of e-Drone on the given dummy.
This node publishes and subsribes the following topics:

		PUBLICATIONS			SUBSCRIPTIONS
		/drone_command			/whycon/poses
		/alt_error				/pid_tuning_altitude
		/pitch_error			/pid_tuning_pitch
		/roll_error				/pid_tuning_roll
		/yaw_error				/pid_tuning_yaw
								/drone_yaw

Rather than using different variables, use list. eg : self.setpoint = [1,2,3,4], where index corresponds to x,y,z and yaw_value...rather than defining self.x_setpoint = 1, self.y_setpoint = 2
CODE MODULARITY AND TECHNIQUES MENTIONED LIKE THIS WILL HELP YOU GAINING MORE MARKS WHILE CODE EVALUATION.	
'''

# Importing the required libraries



'''
* Team Id : eYRC#1082
* Author List : Rahul,Vinayaka,Khoushikh,Sriram
* Filename: 1082_pos_hold.py
* Theme: Hungry Bird
* Functions: disarm,arm,whycon_callback,altitude_set_pid,pitch_set_pid,roll_set_pid,pid
* Global Variables:
'''

from plutodrone.msg import *    
from geometry_msgs.msg import PoseArray
from geometry_msgs.msg import Pose
from std_msgs.msg import Int16
from std_msgs.msg import Int64
from std_msgs.msg import Float64
from std_msgs.msg import Float32MultiArray
from pid_tune.msg import PidTune
import numpy as np
import rospy
import time


class Edrone():
	"""docstring for Edrone"""
		"""docstring for Edrone"""
	
	'''
	* Function Name: __init__
	* Logic: Initialization Function
	'''
	
	def __init__(self):
		
		rospy.init_node('drone_control')	# initializing ros node with name drone_control

	#------------------------------------- Kalman Filter initialization (Explained Later)-----------------------------------------------
		self.A = np.matrix([[1,0,0],[0,1,0],[0,0,1]])  # Process 
		self.B = 0  # Control 
		self.C = np.matrix([[1,0,0],[0,1,0],[0,0,1]])  # Measurement
		self.current_state_estimate = 0  # Current state estimate
		self.current_prob_estimate = np.matrix([[1,0,0],[0,1,0],[0,0,1]])  # Current probability
		self.Q = np.matrix([[0.95,0,0],[0,0.95,0],[0,0,0.005]]) # Process covariance
		self.R = np.matrix([[10,0,0],[0,10,0],[0,0,10]])  # Measurement covariance
	#-------------------------------------------------------------------------------------------------------------------	

		# This corresponds to your current position of drone. This value must be updated each time in your whycon callback
		# [x,y,z,yaw_value]
		self.drone_position = [0.0,0.0,0.0,0.0]	

		self.initial_waypoint = [0,0,0,0]
		self.setpoint = self.initial_waypoint


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
		self.error = [0.0,0.0,0.0,0.0]

		#### Error Margins #####################
		self.margin_x = 2
		self.margin_y = 2
		self.margin_z = 2

		#----------------------------- Initializing Publishers ---------------------------------
		self.alt_error = Float64()          #message type for publishing error values
		self.alt_error.data = self.error[2]

		self.pitch_error = Float64()		 #message type for publishing error values
		self.pitch_error.data = self.error[0]

		self.roll_error = Float64()			 #message type for publishing error values
		self.roll_error.data = self.error[1]


		self.zero_line = Float64()			 #message type for publishing error values
		self.zero_line.data = 0.0

		self.kalman_altitude = Float64()
		self.kalman_pitch = Float64()
		self.kalman_roll = Float64()

		self.normal_altitude = Float64()
		self.normal_pitch = Float64()
		self.normal_roll = Float64()	

		self.filtered_position = Pose()	# to publish Filtered Kalman values to vrep

		self.start_position = Float32MultiArray() # To publish co-ordinates to Dummy in V-reep

		#----------------------------------------------------------------------------------------------------
		
		self.path_request = Int64()
		self.path_request.data = 0;

		self.path_points = PoseArray()

		#initial setting of Kp, Kd and ki for [pitch, roll, throttle, yaw]. eg: self.Kp[2] corresponds to Kp value in throttle axis
		#after tuning and computing corresponding PID parameters, change the parameters

		self.Kp = [8,8,40,0] 
		self.Ki = [0.09,0.09,0,0]
		self.Kd = [17,17,30,0]


		#-----------------------Add other required variables for pid here ----------------------------------------------
		self.prev_errors = [0.0,0.0,0.0,0.0]  # to store previous errors
		self.max_values = [1800,1800,1800,1800]
		self.min_values = [1200,1200,1200,1200]


		

		self.sample_time = 0.008 

		self.ready = 0  # Variable to check if drone is ready to take off






########## to store output of pid equation ##################################
		self.out_pitch = 0  # output values from the pid loop
		self.out_roll = 0
		self.out_throttle = 0
		self.out_yaw = 0
		
		self.proportional = [0,0,0,0]  # variables for the pid loop
		self.integral = [0,0,0,0]
		self.derivative = [0,0,0,0]
		self.error_sum = [0.0,0.0,0.0,0.0] # to store summation of error ( integral of error)


############### Path Logic #######################
		self.first_time = True
		self.counter = 0 # to count through the setpoints
		self.progress = 0 # to track progress of the program
		self.move_flag = False # flag to start or stop following path
		self.total_points = 5 # total setpoints sent by ompl
###################################################		


	#---------------------------------------- Publishers------------------------------------------------
		# Publishing /drone_command, /alt_error, /pitch_error, /roll_error, /yaw_error
		self.command_pub = rospy.Publisher('/drone_command', PlutoMsg, queue_size=1)
		#------------------------Add other ROS Publishers here-----------------------------------------------------
		
		################ Requests path to OMPL #######################################
		self.path_request_pub = rospy.Publisher('/path_request',Int64, queue_size = 1)

		self.alt_error_pub = rospy.Publisher('/alt_error',Float64, queue_size=1)

		self.pitch_error_pub = rospy.Publisher('/pitch_error',Float64, queue_size=1)

		self.roll_error_pub = rospy.Publisher('/roll_error',Float64, queue_size=1)

		self.zero_line_pub = rospy.Publisher('/zero_line_pub',Float64, queue_size = 1)

		
		#------------------------- Kalman stuff -----------------------------------------
		self.kalman_altitude_pub = rospy.Publisher('/kalman_altitude',Float64, queue_size = 1)
		self.kalman_roll_pub = rospy.Publisher('/kalman_roll',Float64, queue_size = 1)
		self.kalman_pitch_pub = rospy.Publisher('/kalman_pitch',Float64, queue_size = 1)


		self.normal_altitude_pub = rospy.Publisher('/normal_altitude',Float64, queue_size = 1)
		self.normal_pitch_pub = rospy.Publisher('/normal_pitch',Float64, queue_size = 1)
		self.normal_roll_pub = rospy.Publisher('/normal_roll',Float64, queue_size = 1)

		self.filtered_position_pub = rospy.Publisher('/filtered_position',Pose, queue_size = 1) # to publish filtered position


		self.start_position_pub = rospy.Publisher('/start_position',Float32MultiArray, queue_size = 1) # to publish start position to vrep


		#----------------------------------------------------------------------------------



		#-----------------------------------------------------------------------------------------------------------


		# Subscribing to /whycon/poses, /drone_yaw, /pid_tuning_altitude, /pid_tuning_pitch, pid_tuning_roll
		rospy.Subscriber('whycon/poses', PoseArray, self.whycon_callback)
		
########################### Subscribes to Topic published by OMPL #############################################
		rospy.Subscriber('/vrep/waypoints', PoseArray , self.update_path)

		rospy.Subscriber('/input_key', Int16, self.input_callback)






		#------------------------------------------------------------------------------------------------------------

	'''
	* Function Name:disarm
	* Logic: Disarms the drone
	'''
	def disarm(self):
		self.cmd.rcAUX4 = 1100
		self.command_pub.publish(self.cmd)
		rospy.sleep(1)


	# Arming condition of the drone : Best practise is to disarm and then arm the drone.
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
		self.whycon = msg
		self.normal_y = msg.poses[0].position.y
		# print(self.normal_y)

		# --------------------Set the remaining co-ordinates of the drone from msg----------------------------------------------

		self.normal_x = msg.poses[0].position.x  # or is it poses[1] or poses[2]

		self.normal_z = msg.poses[0].position.z

		self.filtered_position.position.x = self.drone_position[1]
		self.filtered_position.position.y = self.drone_position[0] 	
		self.filtered_position.position.z = self.drone_position[2] 

		self.filtered_position_pub.publish(self.filtered_position) # publishing filered whycon
		

  #--------------------------------------------------------------------------------------------------------------------------------------------#






   '''
   Function NAme : input_callback

   input : data of key pressed

   Logic : to arm the drone

   '''

	def input_callback(self,msg):
		if msg.data == 70:
			self.ready = 1
		
		#---------------------------------------------------------------------------------------------------------------





	





	#----------------------------------Kalman Filter Begin-----------------------------------------------------
   '''
   Function NAme : Kalman filter code

   input : Raw Whycon data

   Logic : FIlters the raw noisy whycon data to smooth filtered values suitable for PID. By removing the noise present the PID performance is significantly increased 
	
			By removing the noise for PID input, the PID outpout is smooth and the undesirable jerkiness is prevended.  

			I have attached a image to show the difference.
   '''


	def step(self, control_input, measurement): 
	# Prediction step

	'''
	Filter eqautions are implemented using linear algebra. 



	'''
		predicted_state_estimate = np.dot(self.A,self.current_state_estimate)
		predicted_prob_estimate = np.dot(np.dot(self.A,self.current_prob_estimate),self.A) + self.Q

		innovation = measurement - np.dot(self.C,predicted_state_estimate)
		innovation_covariance = np.dot(np.dot(self.C,predicted_prob_estimate), self.C) + self.R
		innovation_covariance = np.linalg.inv(innovation_covariance)
		kalman_gain = np.dot(np.dot(predicted_prob_estimate,self.C),innovation_covariance)
		self.current_state_estimate = predicted_state_estimate + np.dot(kalman_gain,innovation)

		self.current_prob_estimate = np.dot(((np.matrix([[1,0,0],[0,1,0],[0,0,1]]))- np.dot(kalman_gain,self.C)),predicted_prob_estimate)
		return self.current_state_estimate

	





	'''
	Function: update_pos

	Logic : updates Drone position based on Filterd values

	'''


	def update_pos(self):
		whycon_matrix = np.matrix([[self.normal_x],[self.normal_y],[self.normal_z]])
		self.drone_position[0] = test_variable[1,0]
		self.drone_position[1] = test_variable[0,0]
		self.drone_position[2] = test_variable[2,0]

		self.kalman_altitude.data = self.drone_position[2]
		self.normal_altitude.data = self.normal_z

		self.kalman_pitch.data = self.drone_position[0]
		self.normal_pitch.data = self.normal_y

		self.kalman_roll.data = self.drone_position[1]
		self.normal_roll.data = self.normal_x

	


	#------------------------------- Kalman FIlter End -------------------------------------------------------------------








############## Update Setpoints ################################
	def update_path(self,path):

		self.move_flag = True
		self.path_points = path




	#----------------------------------------------------------------------------------------------------------------------

	def pid(self):
		
	#-----------------------------Write the PID algorithm here--------------------------------------------------------------

	# Steps:
	# 	1. Compute error in each axis. eg: error[0] = self.drone_position[0] - self.setpoint[0] ,where error[0] corresponds to error in x...
	#	2. Compute the error (for proportional), change in error (for derivative) and sum of errors (for integral) in each axis. Refer Getting_familiar_with_PID.pdf to understand PID equation.
	#	3. Calculate the pid output required for each axis. For eg: calcuate self.out_roll, self.out_pitch, etc.
	#	4. Reduce or add this computed output value on the avg value ie 1500. For eg: self.cmd.rcRoll = 1500 + self.out_roll. LOOK OUT FOR SIGN (+ or -). EXPERIMENT AND FIND THE CORRECT SIGN
	#	5. Don't run the pid continously. Run the pid only at the a sample time. self.sampletime defined above is for this purpose. THIS IS VERY IMPORTANT.
	#	6. Limit the output value and the final command value between the maximum(1800) and minimum(1200)range before publishing. For eg : if self.cmd.rcPitch > self.max_values[1]:
	#																														self.cmd.rcPitch = self.max_values[1]
	#	7. Update previous errors.eg: self.prev_error[1] = error[1] where index 1 corresponds to that of pitch (eg)
	#	8. Add error_sum


####################################### Calculate PID terms #################################################
		self.update_pos() # Update drone position


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
		# self.out_yaw = self.proportional[3] + self.error_sum[3] + self.derivative[3]       # pid yaw

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


############################### Path Following Logic ############################################################################

		if ((-self.margin_x<=self.error[0]<=self.margin_x) and (-self.margin_y<=self.error[1]<=self.margin_y) and (-self.margin_z<=self.error[2]<=self.margin_z)):
			if self.progress == 0 and self.first_time == True :
				print("waypoint {n} reached".format(n = self.progress))
				self.path_request.data = self.progress
				self.path_request_pub.publish(self.path_request)
				self.first_time = False
				self.initial_yaw = self.drone_position[3]

			if self.counter == (self.total_points):
				print(self.error)
				print(self.drone_position)
				self.progress = self.progress + 1
				self.counter = 0
				self.move_flag = False
				print("waypoint {n} reached".format(n = self.progress))

				if self.progress <= 3:
					self.path_request.data = self.progress
					self.path_request_pub.publish(self.path_request)

			if self.move_flag == True and self.counter < self.total_points :
				self.setpoint = [self.path_points.poses[self.counter].position.y,self.path_points.poses[self.counter].position.x,self.path_points.poses[self.counter].position.z,0]
				self.counter = self.counter + 1 #1 ######################################### WHOAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA HERE ########################
				### printhere
				print(self.counter)

		if self.progress == 4 :

			print("done")
			self.disarm()


		self.command_pub.publish(self.cmd) # publish the commands

		self.kalman_altitude_pub.publish(self.kalman_altitude)
		self.kalman_pitch_pub.publish(self.kalman_pitch)
		self.kalman_roll_pub.publish(self.kalman_roll)


		self.normal_altitude_pub.publish(self.normal_z)
		self.normal_pitch_pub.publish(self.normal_y)
		self.normal_roll_pub.publish(self.normal_x)
		
		self.prev_errors = [self.error[m] for m in range(0,3)] # previous error 
		
		rospy.sleep(self.sample_time) 


if __name__ == '__main__':

	e_drone = Edrone()
	rospy.sleep(2)
	
	######### Initialize parameters for Kalman Filter #####################################

	initial_state = np.matrix([[e_drone.normal_y],[e_drone.normal_x],[e_drone.normal_z]])
	e_drone.current_state_estimate = initial_state
	e_drone.setpoint = [e_drone.whycon.poses[0].position.y,e_drone.whycon.poses[0].position.x,25]
	e_drone.start_position.data = e_drone.setpoint
	e_drone.start_position_pub.publish(e_drone.start_position)
	print(e_drone.setpoint)

	################ Wait for arm key press ##################################################
	while e_drone.ready != 1:
		e_drone.pid()
		

	e_drone.arm()
	# print('ya')

	while not rospy.is_shutdown():
		e_drone.pid()