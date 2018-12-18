#!/usr/bin/env python

import time
import rospy
import sys
import signal
import numpy as np
import math
import datetime
import tr2
from sensor_msgs.msg import Joy
from std_msgs.msg import Float64

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
			
