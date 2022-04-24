#!/usr/bin/env python
# Importing Libraries
import mavros
import rospy
import numpy as np
import math
import mavros_msgs
import time
from mavros_msgs import srv
from mavros_msgs.srv import SetMode
from mavros_msgs.srv import SetModeResponse
from mavros_msgs.srv import CommandBool
from mavros_msgs.srv import CommandTOL
from mavros_msgs.srv import ParamSet
from mavros_msgs.msg import ParamValue
from mavros_msgs.msg import State
from mavros_msgs.msg import OverrideRCIn
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import NavSatFix
from tf.transformations import quaternion_from_euler
from std_msgs.msg import String


alt_set = 2
class Operation(): 
	# Staring class Operation 
	goal_pose = PoseStamped() # renaming PoseStamped presumably end position 
	set_velocity = TwistStamped() # renaming TwistStamped to set velocity to be used as a structure
	current_state = State() # renaming state which relays if FCU is connected to GCU, armed, and guided


	def state_callback(self,data):
		self.cur_state = data
		# Gets state from /mavros/state

	def pose_sub_callback(self,pose_sub_data):
		self.current_pose = pose_sub_data
		# Callback function to get current position for FCU   

	def gps_callback(self,data):
		self.gps = data
		self.gps_read = True
		# Gets GPS data and will set a bool value for read

	def start_operation(self):
		rospy.init_node('my_operation', anonymous=True)	# Starting node for operation 
		self.gps_read = False # When starting gps read will be sent to false initially
		self.localtarget_received = False 
		r = rospy.Rate(10) # Sents rate to 10Hz
		rospy.Subscriber("/mavros/state", State, self.state_callback) # Pulling information for mavros/state and sending it to state callback
		rospy.Subscriber("/mavros/global_position/global", NavSatFix, self.gps_callback) # Pulling GPS information and will not access callback to make gps_read statement true if NavSatFix = -1  
		##rospy.Subscriber("cv_node",CV,self.cv_callback)
		local_position_subscribe = rospy.Subscriber('/mavros/local_position/pose', PoseStamped, self.pose_sub_callback) # Getting current position and sets current_pose to subscribed value
		local_position_pub = rospy.Publisher('/mavros/setpoint_position/local', PoseStamped, queue_size = 10) 
		setpoint_velocity_pub = rospy.Publisher('/mavros/setpoint_velocity/cmd_vel',TwistStamped, queue_size = 10)
		while not self.gps_read:
			r.sleep()
		
		# Creation of all Service Clients
		change_mode = rospy.ServiceProxy('/mavros/set_mode', SetMode) 
		arm = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
		takeoff = rospy.ServiceProxy('/mavros/cmd/takeoff', CommandTOL)
		change_param = rospy.ServiceProxy('/mavros/param/set', ParamSet)
		last_request = rospy.get_rostime()

		# Change mode to GUIDED
		rospy.wait_for_service('/mavros/set_mode')
		try:
			base_mode = 0
			custom_mode = "GUIDED"
			out = change_mode(base_mode, custom_mode)
			print(out)
			if out:
				rospy.loginfo("GUIDED mode set")
			else:
				rospy.loginfo("Failed SetMode")
		except rospy.ServiceException, e:
			rospy.loginfo("Service call failed")
		last_request = rospy.get_rostime() 
		
		while not out:
			r.sleep()
			out = change_mode(base_mode, custom_mode)
			if out:
				rospy.loginfo("setmode send ok value")
			else:
				rospy.loginfo("Failed SetMode")

		# Arm drone
		rospy.wait_for_service('/mavros/cmd/arming')
		try:
			out = arm(True)
			if out:
				rospy.loginfo("Armed")
			else:
				rospy.loginfo("Failed Arming")
		except rospy.ServiceException, e:
			rospy.loginfo("Service call failed")
		last_request = rospy.get_rostime() 
		
		while not out:
			r.sleep()
			out = arm(True)
			if out:
				rospy.loginfo("Armed")
			else:
				rospy.loginfo("Failed Arming")

		# Take off
		current_altitude = self.gps.altitude 
		print('\n Takeoff')
		rospy.wait_for_service('/mavros/cmd/takeoff')
		try:
			latitude = self.gps.latitude
			longitude = self.gps.longitude
			altitude = alt_set
			out = takeoff(min_pitch, yaw, latitude, longitude, altitude)
			if out:
				rospy.loginfo("Took-off")
			else:
				rospy.loginfo("Failed taking-off")
		except rospy.ServiceException, e:
			rospy.loginfo("Service call failed")
		
		# Keep sending take-off messages until received by FCU
		while not out:
			r.sleep()
			out = takeoff(min_pitch, yaw, latitude, longitude, altitude)
			if out:
				rospy.loginfo("Took-off")
			else:
				rospy.loginfo("Failed taking-off")

		while self.gps.altitude< current_altitude+altitude-0.1:
			r.sleep()
			differ = self.gps.altitude - current_altitude
			rospy.loginfo("Waiting to take off, current height %s", differ)

		# Position
		print(self.current_pose.pose.position.x)
		self.goal_pose.header.frame_id = "8"
		self.goal_pose.header.seq = 1
		self.goal_pose.pose.position.x = self.current_pose.pose.position.x + 5
		self.goal_pose.pose.position.y = self.current_pose.pose.position.y + 5
		self.goal_pose.pose.position.z = 3 
		local_position_pub.publish(self.goal_pose)
		rospy.loginfo(self.goal_pose)
   
		time.sleep(2)
		# Landing
		print "\nLanding"
		rospy.wait_for_service('/mavros/cmd/land')
		try:
			landing_cl = rospy.ServiceProxy('/mavros/cmd/land', CommandTOL)
			response = landing_cl(altitude=alt_set, latitude=0, longitude=0, min_pitch=0, yaw=0)
			rospy.loginfo(response)
		except rospy.ServiceException as e:
			print("Landing failed: %s" %e)
			rospy.signal_shutdown(True)
			rospy.loginfo("\n Pilot Takeover!")
if __name__ == '__main__':
	my_operation = Operation()
	my_operation.start_operation()