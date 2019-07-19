#!/usr/bin/env python

import time
import socket
import Queue
import threading
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

q = Queue.Queue()

actuatorIds = []
states = []

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

TAU = math.pi * 2.0

class Packet:
	address = "x0"
	msgId = 0x00
	cmd = 0x00
	length = 0
	params = []
	checksum = 0
	
	_startByte = 0xFF
	
	def __init__(self, addr = "x0"):
		self.address = addr
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
		
	def toList(self):
		self.computeLength()
		
		result = [self._startByte, self.address, self.msgId, self.length, self.cmd]
		
		for p in self.params:
			result.append(p)
		
		result.append(self.checksum)
		return result
    
	def toString(self):
		self.computeLength()
  	
		msgString = ""
		msgString = msgString + str(self.address) + ":"
		msgString = msgString + str(self.msgId) + ",0,"
		msgString = msgString + str(self.length) + ","
		msgString = msgString + str(self.cmd) + ","
  	
		for p in self.params:
			msgString = msgString + str(p) + ","

		return msgString + ";\r\n"
  

class Msgs:
	_msgs = []
	_msgId = 0
	_expirePeriod = 0.250
	
	time.sleep(2)
	
	def __init__(self):
		self._msgs = []
		
	def incrementMsgId(self):
		if (self._msgId < 255):
			self._msgId = self._msgId + 1
		else:
			self._msgId = 0

	def getState(self):
		global actuatorIds, states
		return actuatorIds, states
		
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
				
	def retryPending(self):
		timeThreshold = 3 # seconds
		for idx, msg in enumerate(self._msgs):
			ts = time.time()
			if (ts < msg.expires and msg._calledback == False):
				if (msg._sentOn > time.time() - timeThreshold):
					msg._sent = False
				
	def step(self):
		global states, actuatorIds
		
		for idx, msg in enumerate(self._msgs):
			if (msg.sent() == False):
				msg.packet.msgId = self._msgId
				self.incrementMsgId()
				msg.send()
		self.clean()

class Msg:
	packet = Packet()
	expires = time.time() + 60
	retryOnFailure = False
	_attempt = 0
	_sent = False
	_sentOn = None
	_callback = None
	_calledback = False
	_expirePeriod = 0.500
	_responseReceived = False
	
	def __init__(self, addr, packet, callback = None):
		self.packet = packet
		self._callback = callback
		
	def callback(self, packet):
		if (packet != None):
			self._responseReceived = True
				
		if (self._callback and self._calledback == False):
			self._callback(packet)
			self._calledback = True
			self.expires = time.time() - 60
		
	def sent(self):
		return self._sent
    
	def send(self):
		self._attempt += 1;
		
		if (self._sent == True):
			return
			
		msgString = self.packet.toString().encode()
		q.put(msgString)
			
		self._sent = True
		self._sentOn = time.time()
		self.expires = time.time() + self._expirePeriod
		#print "REQ ->", msgString


server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
server_socket.bind(('', 12345))
server_socket.listen(5)

def handle_client():
	global i, states, actuatorIds
	while True:
		try:
			print "Waiting for connection on port 12345"
			c, addr = server_socket.accept()
			print 'Got connection from', addr
			prev_msg = ''
			while True:
				try:
					msg = q.get_nowait()
					if (msg != prev_msg):
						c.send(msg)
						prev_msg = msg
					else:
						c.send("nc;")
				except Queue.Empty:
					c.send("nc;")

				res = c.recv(1024)
				if res and res != "ns;":
					_states = res.split(";")
					for _state in _states:
						if (len(_state.split(':')) > 1):
							id = _state.split(':')[0]
							state = _state.split(':')[1]

							try:
								state = int(state)
							except:
								pass

							inList = False
							for j in range(len(actuatorIds)):
								if id == actuatorIds[j]:
									states[j] = state
									inList = True

							if inList == False:
								actuatorIds.append(id)
								states.append(state)
		except BaseException as e:
			print('{!r}; restarting thread'.format(e))
		else:
			print('exited normally, bad thread; restarting')


t = threading.Thread(target=handle_client, args=())
t.daemon = True
t.start()

