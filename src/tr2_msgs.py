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

class Packet:
	i2cAddress = 0x00
	msgId = 0x00
	cmd = 0x00
	length = 0
	params = []
	checksum = 0
	
	def __init__(self, addr = 0x00):
		self.i2cAddress = addr
		self.params = []
		return
		
	def addParam(self, p):
		self.params.append(p)
	
	def computeChecksum(self):
		self.checksum = 0
		self.checksum = self.checksum + self.msgId
		self.checksum = self.checksum + self.cmd
		self.checksum = self.checksum + self.length
		
		for p in self.params:
			self.checksum = self.checksum + int(p)
  		
		self.checksum = int(math.floor(self.checksum % 256))
    
	def computeLength(self):
		self.length = 4 + len(self.params)
    
	def toString(self):
		self.computeLength()
		self.computeChecksum()
  	
		msgString = ""
		msgString = msgString + str(self.i2cAddress) + " "
		msgString = msgString + str(self.msgId) + " "
		msgString = msgString + str(self.length) + " "
		msgString = msgString + str(self.cmd) + " "
  	
		for p in self.params:
			msgString = msgString + str(p) + " "
  	
		msgString = msgString + str(self.checksum) + " "
		return msgString + ";"
  

class Msgs:
	_msgs = []
	_msgId = 0
	_expirePeriod = 10
	_arduino = serial.Serial('/dev/ttyACM0',
														baudrate=115200,
														bytesize=serial.EIGHTBITS,
														parity=serial.PARITY_NONE,
														stopbits=serial.STOPBITS_ONE,
														timeout=1)
	
	time.sleep(2)
	
	def __init__(self):
		self._msgs = []
		
	def incrementMsgId(self):
		if (self._msgId < 255):
			self._msgId = self._msgId + 1
		else:
			self._msgId = 0
		
	def add(self, msg):
		self._msgs.append(msg)

	def remove(self, msg):
		for idx, val in enumerate(self._msgs):
			if (val.packet.msgId == msg.packet.msgId):
				del self._msgs[idx]
				self.remove(msg)
				return
				
	def clean(self):
		for idx, msg in enumerate(self._msgs):
			ts = time.time()
			if (ts > msg.expires):
				msg.callback(None)
				self.remove(msg)
				self.clean()
				return
				
	def step(self):
		for idx, msg in enumerate(self._msgs):
			if (msg.sent() == False):
				msg.packet.msgId = self._msgId
				self.incrementMsgId()
				msg.expires = time.time() + self._expirePeriod
				msg.send(self._arduino)
				
		self.listen()
		self.clean()
				
	def listen(self):
		while self._arduino.in_waiting:
			if (self._arduino.isOpen() == False):
				self._arduino.open()
			
			p = self._arduino.read_until(";",32)
			p = p.split(" ")
	
			for idx, val in enumerate(p):
				try:
					p[idx] = int(val)
				except:
					p[idx] = val
				
			packet = Packet()
			packet.msgId = p[0]
			packet.length = p[1]
			packet.cmd = p[2]
			
			i = 0
			while i < packet.length - 3:
				packet.addParam(p[i + 3])
				i += 1
			
			packet.checksum = p[packet.length - 1]
			
			for idx, val in enumerate(self._msgs):
				if (val.packet.msgId == packet.msgId):
					packet.i2cAddress = val.packet.i2cAddress
					print "RES <-", packet.toString()
					val.callback(packet)

class Msg:
	packet = Packet()
	expires = time.time() + 60
	_sent = False
	_callback = None
	_calledback = False
	
	def __init__(self, addr, packet, callback = None):
		self.packet = packet
		self._callback = callback
		
	def callback(self, packet):
		if (self._callback and self._calledback == False):
			self._callback(packet)
			self._calledback = True
			self.expires = time.time() - 60
		
	def sent(self):
		return self._sent
    
	def send(self, arduino):
		if (self._sent == True):
			return
			
		if (arduino.isOpen() == False):
			arduino.open()
			
		msgString = self.packet.toString()
		arduino.write(msgString.encode())
		self._sent = True
		print "REQ ->", msgString

