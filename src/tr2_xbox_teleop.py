#!/usr/bin/env python

import time
import rospy
import sys
import numpy as np
import math
import datetime
from sensor_msgs.msg import Joy
from std_msgs.msg import Float64

## arm
pub_JointArm0 = rospy.Publisher('/tr2/controller/effort/JointArm0/command', Float64, queue_size=10)
pub_JointArm1 = rospy.Publisher('/tr2/controller/effort/JointArm1/command', Float64, queue_size=10)
pub_JointArm2 = rospy.Publisher('/tr2/controller/effort/JointArm2/command', Float64, queue_size=10)
pub_JointArm3 = rospy.Publisher('/tr2/controller/effort/JointArm3/command', Float64, queue_size=10)
pub_JointArm4 = rospy.Publisher('/tr2/controller/effort/JointArm4/command', Float64, queue_size=10)
pub_JointArmGripper = rospy.Publisher('/tr2/controller/effort/JointArmGripper/command', Float64, queue_size=10)

# base
pub_JointBaseWheelL = rospy.Publisher('/tr2/controller/effort/JointBaseWheelL/command', Float64, queue_size=10)
pub_JointBaseWheelR = rospy.Publisher('/tr2/controller/effort/JointBaseWheelR/command', Float64, queue_size=10)

# head
pub_JointHeadTilt = rospy.Publisher('/tr2/controller/effort/JointHeadTilt/command', Float64, queue_size=10)
pub_JointHeadPan = rospy.Publisher('/tr2/controller/effort/JointHeadPan/command', Float64, queue_size=10)

mode = 0 # 0 = right arm, 1 = base
increment = 0.05
wristServoValue = 0.5
gripperServoValue = 0
wristServoLeftValue = 0.5
gripperServoLeftValue = 0

old_time = datetime.datetime.now()
def changeMode():
	global mode, old_time;
	current_time = datetime.datetime.now()
	time_diff = current_time - old_time;
	ms_diff = (time_diff.seconds * 1000) + (time_diff.microseconds / 1000);
	if (ms_diff < 100):
		return;
	if (mode == 0):
		mode = 1
		rospy.loginfo("Mode changed to base control")
		old_time = datetime.datetime.now()
		zero_joints()
	elif (mode == 1):
		mode = 2
		rospy.loginfo("Mode changed to head control")
		old_time = datetime.datetime.now()
		zero_joints()
	elif (mode == 2):
		mode = 0
		rospy.loginfo("Mode changed to arm control")
		old_time = datetime.datetime.now()
		zero_joints()

def zero_joints():
	pub_JointArm0.publish(0)
	pub_JointArm1.publish(0)
	pub_JointArm2.publish(0)
	pub_JointArm3.publish(0)
	pub_JointArm4.publish(0)
	pub_JointArmGripper.publish(0)
	pub_JointBaseWheelL.publish(0)
	pub_JointBaseWheelR.publish(0)
	pub_JointHeadPan.publish(0)
	pub_JointHeadTilt.publish(0)

## right arm

def addWristServoValue():
	global wristServoValue, increment
	if (wristServoValue <= 1 - increment):
		wristServoValue = wristServoValue + increment

def subWristServoValue():
	global wristServoValue, increment
	if (wristServoValue >= 0 + increment):
		wristServoValue = wristServoValue - increment

def addGripperServoValue():
	global gripperServoValue, increment
	if (gripperServoValue <= 1 - increment):
		gripperServoValue = gripperServoValue + increment

def subGripperServoValue():
	global gripperServoValue, increment
	if (gripperServoValue >= 0 + increment):
		gripperServoValue = gripperServoValue - increment

## left arm

def addWristServoLeftValue():
	global wristServoLeftValue, increment
	if (wristServoLeftValue <= 1 - increment):
		wristServoLeftValue = wristServoLeftValue + increment

def subWristServoLeftValue():
	global wristServoLeftValue, increment
	if (wristServoLeftValue >= 0 + increment):
		wristServoLeftValue = wristServoLeftValue - increment

def addGripperServoLeftValue():
	global gripperServoLeftValue, increment
	if (gripperServoLeftValue <= 1 - increment):
		gripperServoLeftValue = gripperServoLeftValue + increment

def subGripperServoLeftValue():
	global gripperServoLeftValue, increment
	if (gripperServoLeftValue >= 0 + increment):
		gripperServoLeftValue = gripperServoLeftValue - increment

def subscriber_callback(data):
	global mode
	publishData = []

	if (data.buttons[6] == 1):
		changeMode()

	if (mode == 0): # right arm
		pub_JointArm0.publish(data.axes[0])
		pub_JointArm1.publish(data.axes[1])
		pub_JointArm2.publish(data.axes[2])
		pub_JointArm3.publish(data.axes[3])
		pub_JointArm4.publish(data.axes[6])
		pub_JointArmGripper.publish(data.axes[7])

		if (data.buttons[0] == 1):
			addWristServoValue()

		if (data.buttons[1] == 1):
			subWristServoValue()

		if (data.buttons[2] == 1):
			addGripperServoValue()

		if (data.buttons[3] == 1):
			subGripperServoValue()

	elif (mode == 1): # base
		leftStick = (data.axes[0], data.axes[1])
		rightStickX = data.axes[2]
		motorValues = getMotorValues(np.array(leftStick), rightStickX)
		pub_JointBaseWheelL.publish(motorValues[0])
		pub_JointBaseWheelR.publish(motorValues[1] * -1)
	elif (mode == 2):
		pub_JointHeadPan.publish(data.axes[0])
		pub_JointHeadTilt.publish(data.axes[1])

# input: numpy array, -1 to 1 value
def getMotorValues(desiredVector, rotationStrength):
	if desiredVector.size == 0 and rotationStrength == 0:
		return (0, 0, 0, 0)

	desiredX = desiredVector.tolist()[0]
	desiredY = desiredVector.tolist()[1]
	desiredMagnitude = abs(desiredX)
	if abs(desiredY) > desiredMagnitude:
		desiredMagnitude = abs(desiredY)
	if abs(rotationStrength) > desiredMagnitude:
		desiredMagnitude = abs(rotationStrength)

	if desiredMagnitude == 0:
		return (0, 0, 0, 0)

	# create rotation matrix
	offset = math.pi / 4
	c, s = np.cos(offset), np.sin(offset)
	rotationMatrix = np.matrix('{} {}; {} {}'.format(c, -s, s, c))

	# create output vector from rotation matrix and desiredVector
	ouput = desiredVector.dot(rotationMatrix).tolist()[0]
	outputX = ouput[0]
	outputY = ouput[1]

	# define motors
	frontLeft = outputX
	frontRight = outputY
	backLeft = outputY
	backRight = outputX

	#manipulate values based on rotation from rotationStrength
	rotationStrength = rotationStrength * 2
	frontLeft = frontLeft + rotationStrength
	frontRight = frontRight - rotationStrength
	backLeft = backLeft + rotationStrength
	backRight = backRight - rotationStrength

	# get output magnitude
	outputMagnitude = abs(frontRight)
	if abs(frontLeft) > outputMagnitude:
		outputMagnitude = abs(frontLeft)
	if abs(backLeft) > outputMagnitude:
		outputMagnitude = abs(backLeft)
	if abs(backRight) > outputMagnitude:
		outputMagnitude = abs(backRight)

	# scale output to match desired magnitude
	scale = desiredMagnitude / outputMagnitude

	frontLeft = frontLeft * scale
	frontRight = frontRight * scale
	backLeft = backLeft * scale
	backRight = backRight * scale

	scaledOutput = (frontRight, frontLeft, backLeft, backRight)

	return scaledOutput


def do_control():
	rospy.init_node('tr2_xbox_teleop', anonymous=True)
	rospy.Subscriber("joy", Joy, subscriber_callback)
	rospy.spin()

if __name__ == '__main__':
	do_control()
