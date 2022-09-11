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
	
	def __init__(self):
		
		rospy.init_node('drone_control')	# initializing ros node with name drone_control

		# This corresponds to your current position of drone. This value must be updated each time in your whycon callback
		# [x,y,z,yaw_value]
		self.drone_position = [0.0,0.0,0.0,0.0]	

		self.initial_waypoint = [5.68,-1.91,33.40,0]
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
		
		
		self.path_request = Int64()
		self.path_request.data = 0;

		self.path_points = PoseArray()

		#initial setting of Kp, Kd and ki for [pitch, roll, throttle, yaw]. eg: self.Kp[2] corresponds to Kp value in throttle axis
		#after tuning and computing corresponding PID parameters, change the parameters
		
		self.Kp = [22,20,37,2.58] 
		self.Ki = [0,0,3,0] 
		self.Kd = [20,20,73,13]

		#-----------------------Add other required variables for pid here ----------------------------------------------
		self.prev_errors = [0.0,0.0,0.0,0.0]  # to store previous errors
		self.max_values = [1800,1800,1800,1800]
		self.min_values = [1200,1200,1200,1200]

		# Hint : Add variables for storing previous errors in each axis, like self.prev_values = [0,0,0,0] where corresponds to [pitch, roll, throttle, yaw]
		#		 Add variables for limiting the values like self.max_values = [1800,1800,1800,1800] corresponding to [pitch, roll, throttle, yaw]
		#													self.min_values = [1200,1200,1200,1200] corresponding to [pitch, roll, throttle, yaw]
		#																	You can change the upper limit and lower limit accordingly. 
		#----------------------------------------------------------------------------------------------------------

		# This is the sample time in which you need to run pid. Choose any time which you seem fit. Remember the stimulation step time is 50 ms
		self.sample_time = 0.1 # 0.1 in seconds

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
		self.total_points = 50 # total setpoints sent by ompl
		



		# Publishing /drone_command, /alt_error, /pitch_error, /roll_error, /yaw_error
		self.command_pub = rospy.Publisher('/drone_command', PlutoMsg, queue_size=1)
		#------------------------Add other ROS Publishers here-----------------------------------------------------
		
		################ Requests path to OMPL #######################################
		self.path_request_pub = rospy.Publisher('/path_request',Int64, queue_size = 1)





		#-----------------------------------------------------------------------------------------------------------


		# Subscribing to /whycon/poses, /drone_yaw, /pid_tuning_altitude, /pid_tuning_pitch, pid_tuning_roll
		rospy.Subscriber('whycon/poses', PoseArray, self.whycon_callback)
		
########################### Subscribes to Topic published by OMPL #############################################
		rospy.Subscriber('/vrep/waypoints', PoseArray , self.update_path)






		#------------------------------------------------------------------------------------------------------------

		self.arm() # ARMING THE DRONE


	# Disarming condition of the drone
	def disarm(self):
		self.cmd.rcAUX4 = 1100
		self.command_pub.publish(self.cmd)
		rospy.sleep(1)


	# Arming condition of the drone : Best practise is to disarm and then arm the drone.
	def arm(self):

		self.disarm()

		self.cmd.rcRoll = 1500
		self.cmd.rcYaw = 1500
		self.cmd.rcPitch = 1500
		self.cmd.rcThrottle = 1000
		self.cmd.rcAUX4 = 1500
		self.command_pub.publish(self.cmd)	# Publishing /drone_command
		rospy.sleep(1)



	# Whycon callback function
	# The function gets executed each time when /whycon node publishes /whycon/poses 
	def whycon_callback(self,msg):
		self.drone_position[0] = msg.poses[0].position.x
		# print(msg)

		# --------------------Set the remaining co-ordinates of the drone from msg----------------------------------------------

		self.drone_position[1] = msg.poses[0].position.y  # or is it poses[1] or poses[2]

		self.drone_position[2] = msg.poses[0].position.z





		
		#---------------------------------------------------------------------------------------------------------------
	def drone_yaw_callback(self,msg):

		self.drone_position[3] = msg.data  


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
		self.error = [(self.setpoint[a]-self.drone_position[a]) for a in range(0,4)]  # calculate error array

		self.error_sum = [((self.error_sum[b]+self.error[b])*self.Ki[b]*self.sample_time) for b in range(0,4)] # calculate sum of errors

		self.derivative = [((float(self.error[j]-self.prev_errors[j])/self.sample_time)*self.Kd[j]) for j in range(0,4)] #calculate derivative of errors
 
		self.proportional = [(self.error[n] * self.Kp[n]) for n in range(0,4)] # proportional term in pid equation

		for i in range(len(self.error_sum)):               # limit error_sum to max value
			if self.error_sum[i] > self.max_values[i] :
				self.error_sum[i] = self.max_values[i]

		self.out_pitch = self.proportional[0] + self.error_sum[0] + self.derivative[0]     # pid pitch
		self.out_roll = self.proportional[1] + self.error_sum[1] + self.derivative[1]      # pid roll
		self.out_throttle = self.proportional[2] + self.error_sum[2] + self.derivative[2]  # pid throttle
		self.out_yaw = self.proportional[3] + self.error_sum[3] + self.derivative[3]       # pid yaw

		self.cmd.rcPitch = 1500 - self.out_pitch			# output Pitch
		self.cmd.rcRoll = 1500 - self.out_roll				# output Roll	
		self.cmd.rcThrottle = 1500 - self.out_throttle		# output Throttle
		self.cmd.rcYaw = 1500 + self.out_yaw				# output Yaw


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

		if self.cmd.rcYaw > self.max_values[3]:
			self.cmd.rcYaw = self.max_values[3]
		if self.cmd.rcYaw < self.min_values[3]:
			self.cmd.rcYaw = self.min_values[3]


############################### Path Following Logic ############################################################################

		if ((-0.5<=self.error[0]<=0.5) and (-0.5<=self.error[1]<=0.5) and (-0.5<=self.error[2]<=0.5) and (-0.5<=self.error[3]<=0.5)):
			if self.progress == 0 and self.first_time == True :
				print("waypoint {n} reached".format(n = self.progress))
				self.path_request.data = self.progress
				self.path_request_pub.publish(self.path_request)
				self.first_time = False
				self.initial_yaw = self.drone_position[3]

			if self.counter == (self.total_points):
				self.progress = self.progress + 1
				self.counter = 0
				self.move_flag = False
				print("waypoint {n} reached".format(n = self.progress))

				if self.progress <= 2:
					self.path_request.data = self.progress
					self.path_request_pub.publish(self.path_request)

			if self.move_flag == True and self.counter < self.total_points :
				self.setpoint = [self.path_points.poses[self.counter].position.x,self.path_points.poses[self.counter].position.y,self.path_points.poses[self.counter].position.z,0]
				self.counter = self.counter + 1 #1

		if self.progress == 3 :
			print("disarm")
			self.disarm()


		self.command_pub.publish(self.cmd) # publish the commands
		
		self.prev_errors = [self.error[m] for m in range(0,4)] # previous error 
		
		rospy.sleep(self.sample_time) 


if __name__ == '__main__':

	e_drone = Edrone()

	while not rospy.is_shutdown():
		e_drone.pid()