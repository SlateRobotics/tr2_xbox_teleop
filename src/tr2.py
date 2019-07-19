#!/usr/bin/env python

import time
import rospy
import sys
import signal
import numpy as np
import math
import serial
import datetime
import tr2_msgs
from sensor_msgs.msg import Joy
from std_msgs.msg import Float64

RES_OK = 0x20
RES_ERR = 0x21
RES_OK_POS = 0x22
RES_OK_TRQ = 0x23

CMD_SET_MODE = 0x10
CMD_SET_POS = 0x11
CMD_RESET_POS = 0x12
CMD_ROTATE = 0x13
CMD_RETURN_STATUS = 0x14
CMD_STOP_RELEASE = 0x15
CMD_STOP_EMERGENCY = 0x16

ERR_NONE = 0x00
ERR_CHECKSUM = 0x01
ERR_CMD_BOUNDS = 0x02
ERR_PARAM_BOUNDS = 0x03
ERR_LENGTH = 0x04
ERR_NO_RESPONSE = 0x05
ERR_OTHER = 0xFF

TAU = math.pi * 2.0

class TR2:
	_msgs = tr2_msgs.Msgs()
	
	_state = []
	_stateTS = []
	_stateExpirePeriod = 10 # seconds
	
	_joints = ["b0", "a0", "a1", "a2", "a3", "a4", "g0", "h0", "h1"]
	_jointNames = ["Base", "Arm 0", "Arm 1", "Arm 2", "Arm 3", "Arm 4", "Gripper", "Head Pan", "Head Tilt"]

	_cbs = 0
	_grabbingState = False
	_stopping = False
	
	_minMotorValue = 0.20
	_lastCmdActuate = 0
	_lastCmdDriveLeft = 0
	_lastCmdDriveRight = 0

	def __init__(self):
		i = 0
		while i < len(self._joints):
			self._state.append(None)
			self._stateTS.append(time.time())
			i += 1
		print "TR2 Ready"
			
	def state(self):
		return self._msgs.getState()
    
	def getJoints(self):
		return self._joints
		
	def getJointNames(self):
		return self._jointNames
		
	def setPosition(self, jointIdx, pos, speed = 100):
		x = pos / (math.pi * 2) * 65535
	
		packet = tr2_msgs.Packet()
		packet.address = self._joints[jointIdx]
		packet.cmd = CMD_SET_POS
		packet.addParam(int(math.floor(x % 256)))
		packet.addParam(int(math.floor(x / 256)))
		packet.addParam(int(math.floor(speed / 100.0 * 255.0)))
		
		msg = tr2_msgs.Msg(self._joints[jointIdx], packet)
		self._msgs.add(msg)
		
	def drive(self, motorLeft, motorRight, motorDuration = 750):
		if (abs(motorLeft) <= self._minMotorValue and abs(motorRight) <= self._minMotorValue and abs(self._lastCmdDriveLeft) <= self._minMotorValue and abs(self._lastCmdDriveRight) <= self._minMotorValue):
			self._lastCmdDriveLeft = motorLeft
			self._lastCmdDriveRight = motorRight
			return
		
		self._lastCmdDriveLeft = motorLeft
		self._lastCmdDriveRight = motorRight
	
		jointIdx = 0
		offsetBinary = 100
			
		packet = tr2_msgs.Packet()
		packet.address = self._joints[jointIdx]
		packet.cmd = CMD_ROTATE
		packet.addParam(int((motorLeft * 100.0) + offsetBinary))
		packet.addParam(int((motorRight * 100.0) + offsetBinary))
		packet.addParam(int(math.floor(motorDuration % 256)))
		packet.addParam(int(math.floor(motorDuration / 256)))
		
		msg = tr2_msgs.Msg(self._joints[jointIdx], packet)
		self._msgs.add(msg)

	def releaseAll(self):
		i = 8
		while i >= 1:#len(self._joints):
			self.release(i)
			self.step()
			time.sleep(0.020)
			i -= 1
		
	def stopAll(self):
		i = 8
		while i >= 1:#len(self._joints):
			self.stop(i)
			self.step()
			time.sleep(0.020)
			i -= 1

	def release(self, jointIdx = -1):
		if (jointIdx == -1):
			self.releaseAll()
			
		cmd = CMD_STOP_RELEASE
		
		packet = tr2_msgs.Packet()
		packet.address = self._joints[jointIdx]
		packet.cmd = cmd
		
		msg = tr2_msgs.Msg(self._joints[jointIdx], packet)
		msg.retryOnFailure = True
		self._msgs.add(msg)
		
	def stop(self, jointIdx = -1):
		if (jointIdx == -1):
			self.stopAll()
			
		cmd = CMD_STOP_EMERGENCY
		
		packet = tr2_msgs.Packet()
		packet.address = self._joints[jointIdx]
		packet.cmd = cmd
		
		msg = tr2_msgs.Msg(self._joints[jointIdx], packet)
		msg.retryOnFailure = True
		self._msgs.add(msg)
		
	def actuate(self, jointIdx, motorValue, motorDuration = 500):
		if (abs(motorValue) <= self._minMotorValue and abs(self._lastCmdActuate) <= self._minMotorValue):
			self._lastCmdActuate = motorValue
			return
			
		self._lastCmdActuate = motorValue
			
		offsetBinary = 128
		x = int(math.floor(motorValue * 100.0))
			
		packet = tr2_msgs.Packet()
		packet.address = self._joints[jointIdx]
		packet.cmd = CMD_ROTATE
		packet.addParam(x + offsetBinary)
		packet.addParam(int(math.floor(motorDuration % 256)))
		packet.addParam(int(math.floor(motorDuration / 256)))
		
		msg = tr2_msgs.Msg(self._joints[jointIdx], packet)
		self._msgs.add(msg)
	
	def resetEncoderPosition(self, jointIdx):
		packet = tr2_msgs.Packet()
		packet.address = self._joints[jointIdx]
		packet.cmd = CMD_RESET_POS
		
		msg = tr2_msgs.Msg(self._joints[jointIdx], packet)
		msg.retryOnFailure = True
		self._msgs.add(msg)
		
	def setModeAll(self, mode, j = range(len(_joints))):
		for i in j:
			self.setMode(i, mode)
			self.step()
			time.sleep(0.050)
		
	def setMode(self, jointIdx, mode):
		packet = tr2_msgs.Packet()
		packet.address = self._joints[jointIdx]
		packet.cmd = CMD_SET_MODE
		packet.addParam(mode)
		
		msg = tr2_msgs.Msg(self._joints[jointIdx], packet)
		msg.retryOnFailure = True
		self._msgs.add(msg)
    
	def step(self):
		self._msgs.step()
		
		# expire any state measurements outside of expire period
		for idx, val in enumerate(self._state):
			tsdif = time.time() - self._stateTS[idx]
			if (tsdif > self._stateExpirePeriod):
				self._state[idx] = None
				self._stateTS[idx] = time.time()
		
  	
	def spin(self, condition = True):
		print "TR2 Ready"
		while condition == True:
			self.step()
			time.sleep(0.1)
		self.close()
			
	def close(self):
		self._arduino.close()
