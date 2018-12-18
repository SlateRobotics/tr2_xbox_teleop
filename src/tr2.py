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

TAU = math.pi * 2.0

class TR2:
	_msgs = tr2_msgs.Msgs()
	_joints = [0x70, 0x10, 0x11, 0x12, 0x13, 0x14, 0x30, 0x20, 0x21]
	_jointNames = ["Base", "Arm 0", "Arm 1", "Arm 2", "Arm 3", "Arm 4", "Gripper", "Head Pan", "Head Tilt"]

	def __init__(self):
		i = 0
    
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
		
	def getState(self, jointIdx):
		packet = tr2_msgs.Packet()
		packet.i2cAddress = self._joints[jointIdx]
		packet.cmd = CMD_RETURN_STATUS
		
		def callback(msg):
			print "callback", msg.packet.toString()
		
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
