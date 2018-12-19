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
from sensor_msgs.msg import Joy
from std_msgs.msg import Float64


selectedJoint = 0
joints = [0x70, 0x10, 0x11, 0x12, 0x13, 0x14, 0x30, 0x20, 0x21]
jointNames = ["Base", "Arm 0", "Arm 1", "Arm 2", "Arm 3", "Arm 4", "Gripper", "Head Pan", "Head Tilt"]

mode = 0
modes = [0x10, 0x11, 0x12]
modeNames = ["Servo", "Backdrive", "Rotate"]

close = False
tr2 = tr2.TR2()
lastJoyUpdate = time.time()

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
	global mode, lastJoyUpdate
	
	# ignore if too soon from previous
	timeSince = time.time() - lastJoyUpdate
	if (timeSince * 1000 < 200):
		return
	else:
		lastJoyUpdate = time.time()
	
	# Select/Back: change joint
	if (data.buttons[6] == 1):
		changeJoint()
		
	# Start button: Reset encoder
	if (data.buttons[7] == 1):
		tr2.resetEncoderPosition()
		
	# A button: record position
	if (data.buttons[0] == 1):
		play_recording()
		
	# B button: play positions
	if (data.buttons[1] == 1):
		record_pos()
		
	# X button: get all states
	if (data.buttons[2] == 1):
		record_state()
		
	# Y button: reset positions
	if (data.buttons[3] == 1):
		reset_pos()
		reset_state()
		
	# Both joysticks clicked in: Go joint home
	if (data.buttons[9] == 1 and data.buttons[10] == 1): 
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
		leftStick = (data.axes[0], data.axes[1])
		rightStickX = data.axes[2]
		motorValues = tr2_utils.getMotorValues(np.array(leftStick), rightStickX)
		#pub_JointBaseWheelL.publish(motorValues[0])
		#pub_JointBaseWheelR.publish(motorValues[1] * -1)
	else:
		if (mode == 0): # Servo Mode
			if (data.axes[0] < 0.20 and data.axes[0] > -0.20):
				return
			pos = data.axes[0] * math.pi
			if (pos < 0):
				pos = pos + (math.pi * 2.0)
			tr2.setPosition(selectedJoint, pos)
		elif (mode == 1): # Backdrive Mode
			return
		elif (mode == 2): # Rotate Mode
			tr2.actuate(selectedJoint, data.axes[0], 15000)
			
recordedPos = []
lastPosRec = time.time()
def record_pos():
	global lastPosRec, recordedPos
	def callback(pos, err = None):
		global lastPosRec, recordedPos
		if (err == None):
			tsdif = time.time() - lastPosRec
			recordedPos.append([pos,tsdif])
			lastPosRec = time.time()
			print "Added position", len(recordedPos), "-", recordedPos
	tr2.getState(selectedJoint, callback)
	
def reset_pos():
	global lastPosRec, recordedPos
	print "Reset positions"
	recordedPos = []
	lastPosRec = time.time()
	
recordedState = [] # [[state, timestamp]]
lastStateRec = time.time()
def record_state():
	global lastStateRec, recordedState
	def callback(err = False):
		if (err == True):
			print "Err"
			return
		tsdif = time.time() - lastStateRec
		recordedState.append([tr2.state()[0], tsdif])
		print recordedState
	tr2.getStateAll(callback)
	
def reset_state():
	global lastStateRec, recordedState
	print "Reset states"
	recordedState = []
	lastStateRec = time.time()
	
def play_recording():
	print "Playing positions"
	tr2.setMode(selectedJoint, modes[0])
	i = 0
	while i < len(recordedPos):
		tr2.setPosition(selectedJoint, recordedPos[i][0])
		if (i + 1 < len(recordedPos)):
			time.sleep(recordedPos[i+1][1])
		i += 1
	print "Position playback complete"

def teleop():
	global close
	rospy.init_node('tr2_xbox_teleop', anonymous=True)
	rospy.Subscriber("joy", Joy, subscriber_callback)
	
	while close != True:
		tr2.step()
		time.sleep(0.05)
		
	tr2.close()

if __name__ == '__main__':
	teleop()
