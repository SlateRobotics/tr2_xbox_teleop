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
CMD_SET_FREQUENCY = 0x15

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
	_joints = [0x70, 0x10, 0x11, 0x12, 0x13, 0x14, 0x30, 0x20, 0x21]
	_jointNames = ["Base", "Arm 0", "Arm 1", "Arm 2", "Arm 3", "Arm 4", "Gripper", "Head Pan", "Head Tilt"]

	_cbs = 0
	_grabbingState = False

	def __init__(self):
		i = 0
		while i < len(self._joints):
			self._state.append(None)
			self._stateTS.append(time.time())
			i += 1
			
	def state(self):
		return self._state, self._stateTS
    
	def getJoints(self):
		return self._joints
		
	def getJointNames(self):
		return self._jointNames
		
	def setPosition(self, jointIdx, pos):			
		x = pos / (math.pi * 2) * 65535
	
		packet = tr2_msgs.Packet()
		packet.i2cAddress = self._joints[jointIdx]
		packet.cmd = CMD_SET_POS
		packet.addParam(int(math.floor(x % 256)))
		packet.addParam(int(math.floor(x / 256)))
		
		msg = tr2_msgs.Msg(self._joints[jointIdx], packet)
		self._msgs.add(msg)
		
	def actuate(self, jointIdx, motorValue, motorDuration):
		offsetBinary = 128
		x = int(math.floor(motorValue * 100.0))
			
		packet = tr2_msgs.Packet()
		packet.i2cAddress = self._joints[jointIdx]
		packet.cmd = CMD_ROTATE
		packet.addParam(x + offsetBinary)
		packet.addParam(int(math.floor(motorDuration % 256)))
		packet.addParam(int(math.floor(motorDuration / 256)))
		
		msg = tr2_msgs.Msg(self._joints[jointIdx], packet)
		self._msgs.add(msg)
	
	def resetEncoderPosition(self, jointIdx):
		packet = tr2_msgs.Packet()
		packet.i2cAddress = self._joints[jointIdx]
		packet.cmd = CMD_RESET_POS
		
		msg = tr2_msgs.Msg(self._joints[jointIdx], packet)
		self._msgs.add(msg)
		
	def setMode(self, jointIdx, mode):
		packet = tr2_msgs.Packet()
		packet.i2cAddress = self._joints[jointIdx]
		packet.cmd = CMD_SET_MODE
		packet.addParam(mode)
		
		msg = tr2_msgs.Msg(self._joints[jointIdx], packet)
		self._msgs.add(msg)
		
	def getStateAll(self, callback = None):
		if (self._grabbingState == True):
			return callback(err = True)
	
		i = 2
		self._cbs = 0
		self._grabbingState = True
		while i < 5:#len(self._joints):
			def cb(pos, err = None, idx = i):
				self._cbs += 1
				self._state[idx] = pos
				self._stateTS[idx] = time.time()
				if (self._cbs >= 3):
					self._grabbingState = False
					callback()
			self.getState(i, cb)
			i += 1
		
	def getState(self, jointIdx, cb):
		packet = tr2_msgs.Packet()
		packet.i2cAddress = self._joints[jointIdx]
		packet.cmd = CMD_RETURN_STATUS
		
		def callback(packet):
			if packet == None:
				cb(None, ERR_NO_RESPONSE)
			elif (packet.cmd == RES_OK_POS):
				pos = (packet.params[0] + packet.params[1] * 256.0) / 65536.0 * math.pi * 2
				cb(pos)
			else:
				cb(None, ERR_OTHER)
		
		msg = tr2_msgs.Msg(self._joints[jointIdx], packet, callback)
		self._msgs.add(msg)
    
	def step(self):
		self._msgs.step()
  	
	def spin(self):
		while True:
			self.step()
			time.sleep(0.1)
			
	def close(self):
		self._arduino.close()
