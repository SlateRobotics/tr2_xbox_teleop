#!/usr/bin/env python

import time
import rospy
import sys
import signal
import numpy as np
import math
import datetime
import tr2
import tr2_utils
import tr2_tasks
from sensor_msgs.msg import Joy
from std_msgs.msg import Float64

selectedJoint = 0
joints = [0x70, 0x10, 0x11, 0x12, 0x13, 0x14, 0x30, 0x20, 0x21]
jointNames = ["Base", "Arm 0", "Arm 1", "Arm 2", "Arm 3", "Arm 4", "Gripper", "Head Pan", "Head Tilt"]
jointIds = ["b0", "a0", "a1", "a2", "a3", "a4", "g0", "h0", "h1"]

mode = 0
modes = [0x10, 0x11, 0x12]
modeNames = ["Servo", "Backdrive", "Rotate"]

close = False
tr2 = tr2.TR2()
lastJoyUpdate = time.time()

joy_data = None

waypoints = [] # [[(actuatorId, position)]]
e_stop = False

rpm = 3.75
recordedState = []
recordedState = tr2_tasks.pickAndPlaceSingle
recordedState = tr2_tasks.keurigOperator

def signal_handler(sig, frame):
	global close
	close = True
	sys.exit(0)
	
signal.signal(signal.SIGINT, signal_handler)

def changeJoint():
	global selectedJoint, mode, lastJointChange;
	maxJoint = len(joints)
	if (selectedJoint >= maxJoint - 1):
		selectedJoint = 0
	else:
		selectedJoint += 1
	
	name = jointNames[selectedJoint]
	rospy.loginfo("Joint changed to " + name)
	changeJointTS = datetime.datetime.now()
	
	mode = -1
	lastJointChange = time.time()
	
def changeMode(m):
	global selectedJoint, mode;
	mode = m
	tr2.setMode(selectedJoint, modes[m])
	rospy.loginfo("Mode changed to " + modeNames[mode])
	lastModeChange = time.time()
	
def subscriber_callback(data):
	global joy_data, e_stop
	joy_data = data
	
	# XBOX-Button: emergency stop
	if (data.buttons[8] == 1):
		e_stop = True
		tr2.stopAll()
		time.sleep(1.000)
		tr2.setModeAll(modes[2]) # rotate
		time.sleep(0.300)
		
def teleop():
	global mode, joy_data, e_stop
	
	if (joy_data == None):
		return
		
	data = joy_data

	# B button: undo-emergency stop
	if (data.buttons[1] == 1):
		print "Releasing actuators!"
		e_stop = False
		tr2.releaseAll();
	# XBOX-Button: emergency stop
	if (data.buttons[8] == 1):
		print "Executing emergency stop!"
		e_stop = True
		tr2.stopAll()
		time.sleep(1.000)
		tr2.setModeAll(modes[2]) # rotate
		time.sleep(0.300)
	
	# Left trigger: set all joints backdrive
	if (data.axes[5] < 0):
		print "Setting all joints to backdrive mode"
		tr2.setModeAll(modes[1], (1,2,3,4,5))

	# Right trigger: set all joints servo
	if (data.axes[4] < 0):
		print "Setting all joints to servo mode"
		tr2.setModeAll(modes[0], (1,2,3,4,5))

	# Select/Back: change joint
	if (data.buttons[6] == 1):
		changeJoint()
	
	# Start button: Reset actuator torque and position offsets
	if (data.buttons[7] == 1):
		print("Resetting encoder positions")
		tr2.resetEncoderPosition(selectedJoint)
	
	# A button: playback position waypoints
	if (data.buttons[0] == 1):
		playback_waypoints()
	
	# X button: record position waypoint
	if (data.buttons[2] == 1):
		record_waypoint()
	
	# Y button: reset all position waypoints
	if (data.buttons[3] == 1):
		reset_waypoints()
		
	# Both joysticks clicked in: Go joint home
	if (data.buttons[9] == 1 and data.buttons[10] == 1):
		print("Sending home")
		if (mode != 0): # mode servo
			changeMode(0)
		tr2.setPosition(selectedJoint, 0)
	
	# LB/RB buttons: change mode
	if (data.buttons[4] == 1): 
		if (mode > 0):
			changeMode(mode - 1)
		else:
			changeMode(2)
	elif (data.buttons[5] == 1):
		if (mode < 2):
			changeMode(mode + 1)
		else:
			changeMode(0)

	if (joints[selectedJoint] == 0x70): # base
		y = data.axes[1]
		rotation = data.axes[2]
		if (abs(data.axes[0]) > abs(rotation)):
			rotation = -1.0 * data.axes[0]
		motorValues = tr2_utils.getMotorValues(y, rotation)
		tr2.drive(motorValues[0], motorValues[1])
	else:
		if (mode == 0): # Servo Mode
			if (data.axes[0] < 0.20 and data.axes[0] > -0.20):
				return
			pos = data.axes[0] * math.pi

			if (joints[selectedJoint] == 0x30): # base
				if pos > 0:
					pos = 1
				elif pos < 0:
					pos = 0
				tr2.setPosition(selectedJoint, pos)
			else:
				if (pos < 0):
					pos = pos + (math.pi * 2.0)
			tr2.setPosition(selectedJoint, pos)
		elif (mode == 1): # Backdrive Mode
			return
		elif (mode == 2): # Rotate Mode
			tr2.actuate(selectedJoint, data.axes[0])

# returns current pos in waypoint format
# ex: [[('a0', 3.14), ('a1', 1.25)]]
def get_waypoint():
	_actuatorIds, _state = tr2.state()
	w = []
	for i in range(len(_actuatorIds)):
		pos = ''
		try:
			pos = float(_state[i])
			w.append((_actuatorIds[i], pos))
		except:
			pass
	return w

def get_waypoint_dif(w1, w2):
	w_diff = []
	for s1 in w1:
		aid1 = s1[0]
		pos1 = s1[1]
		found = False
		for s2 in w2:
			aid2 = s2[0]
			pos2 = s2[1]
			if (aid2 == aid1):
				found = True
				diff = math.atan2(math.sin(pos1 - pos2), math.cos(pos1 - pos2))
				w_diff.append((aid1, round(abs(diff), 4)))
	return w_diff
			
def record_waypoint():
	global waypoints
	w = get_waypoint()
	waypoints.append(w)
	f = open("/home/nvidia/ros_ws/src/tr2_xbox_teleop/config/_waypoints.txt", "w")
	f.write(str(waypoints))
	f.close()
	print "Waypoint added: ", w
	
def reset_waypoints():
	global waypoints
	print "Waypoints reset to []"
	waypoints = []
	
def playback_waypoints():
	global waypoints, e_stop, joy_data

	print "Playing back recorded waypoints (" + str(len(waypoints)) + ")"
	
	print "Setting actuators to servo mode"
	tr2.setModeAll(modes[0])
	time.sleep(0.300)

	w_num = -1
	for w in waypoints:
		if e_stop == True:
			return

		w_num = w_num + 1
		g_wait = False

		#get waypoint dif from current pos
		w_cur = get_waypoint()
		w_dif = get_waypoint_dif(w_cur, w)
		min_dif = 3.14
		max_dif = 0.00
		for s in w_dif:
			aid = s[0]
			if aid.startswith('g') and abs(s[1]) > 0.9:
				g_wait = True
			if aid.startswith('a') or aid.startswith('g') or aid.startswith('h'):
				dif = abs(s[1])
				if dif > max_dif:
					max_dif = dif
				if dif < min_dif:
					min_dif = dif

		# calculate max time needed to get to pos
		max_time_s = 0
		for s in w_dif:
			# calculate actuator max speed
			motor_rpm = 9.0

			if s[0] == 'a1':
				motor_rpm = 11.25

			actuator_reduction = 3.0
			actuator_rpm = motor_rpm / actuator_reduction # rotations per minute
			actuator_rps = actuator_rpm * math.pi * 2.0 / 60.0 # radians per second

			if s[0].startswith('a') or s[0].startswith('h'):
				pos = abs(s[1])
				t = pos * actuator_rps # seconds to get there
				if t > max_time_s:
					max_time_s = t

		# set speeds based such that they arrive in max_time_s
		min_speed = 30 # 0 rpm
		max_speed = 100 # actuator_rpm
		for s in w_dif:
			aid = s[0]
			if aid.startswith('a') or aid.startswith('h'):
				pos = abs(s[1])
				t = pos * actuator_rps # seconds to get there
				t_scale = math.floor(t / max_time_s * 100.0)
				speed = ((t_scale * (max_speed - min_speed)) / 100.0) + min_speed

				for i in range(len(w)):
					_s = w[i]
					if _s[0] == s[0]:
						w[i] = (_s[0], _s[1], speed)

		print "Moving to waypoint " + str(w_num)
		for s in w:
			actuatorId = s[0]
			pos = s[1]
			speed = 100

			if len(s) > 2:
				speed = s[2]

			for i in range(len(jointIds)):
				if (jointIds[i] == actuatorId):
					speed = 100
					tr2.setPosition(i, pos, speed)
					tr2.step()
					time.sleep(0.050)

		#calculate waypoint completion
		w_complete = False
		w_t_start = datetime.datetime.now()
		while w_complete == False:
			if e_stop == True:
				return

			t_cur = datetime.datetime.now()
			w_t_dif = t_cur - w_t_start
			if (w_t_dif.total_seconds() > 8.00):
				print("Waypoint " + str(w_num) + " took longer than 8 sec to complete.")
				break

			if joy_data.buttons[5] == 1:
				print("Next button pressed. Moving on.")
				w_complete = True
				break

			w_cur = get_waypoint()
			w_dif = get_waypoint_dif(w_cur, w)
			w_complete = True
			for s in w_dif:
				not_act = s[0].startswith('b')
				not_act = s[0].startswith('s')
				if not_act:
					break
				if g_wait == True:
					g_wait = False
					time.sleep(10.0)
				elif abs(s[1]) > 0.02:
					w_complete = False
		
		print("Waypoint " + str(w_num) + " complete")
	
	print "Waypoint playback complete"

def program():
	global close, waypoints
	rospy.init_node('tr2_xbox_teleop', anonymous=True)
	rospy.Subscriber("joy", Joy, subscriber_callback)

	f = open("/home/nvidia/ros_ws/src/tr2_xbox_teleop/config/_waypoints.txt", "r+")
	waypoints = eval(f.read())
	print("Waypoints set from file (" + str(len(waypoints))) + ")"
	f.close()

	tr2.setModeAll(modes[2]) # rotate
	time.sleep(0.300)
	tr2.releaseAll()
	time.sleep(0.300)

	while close != True:
		teleop()
		tr2.step()
		time.sleep(0.150)

if __name__ == '__main__':
	program()
