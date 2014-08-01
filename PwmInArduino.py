#!/usr/bin/python
#reads pwm input from the arduino via serial
from bbio import *
import os, sys, time

class Arduino:
	def __init__(self, serial, baud):
		self.serial=serial
		self.baudrate=baud
		self.serial.begin(self.baudrate)
		self.data=None
	def __del__(self):
		self.serial.end()
	def getData(self):
		return self.data
	def acquire(self, delimeter):
		self.serial.flush()
		self.data=''
		byte=''
		while (self.serial.read()!='S'):
			pass
		while (byte!='T'):
			byte=self.serial.read()
			if (byte=='T'):
				break
			self.data+=byte
		if self.data == '':
			return None
		self.data=[float(i) for i in self.data.split(delimeter)]
	'''def acquire(self, delimeter):
		self.serial.flush()
		self.serial.println("GET")  #sends GET\r\n
		data=''
		#please format data as "<val><delimeter><val><delimeter>..."
		while (self.serial.available()):
			data+=self.serial.read()
			delay(5)
		print data
		if data == '':
			return None
		return [float(i) for i in data.split(delimeter)]'''

if (__name__=="__main__"):
	ard=None
	timestamp=time.time()
	printtime=time.time()
	def setup():
		global ard
		ard=Arduino(Serial2, 9600)
	def loop():
		global ard
		global timestamp
		global printtime
		curtime=time.time()
		if (curtime-timestamp>0.001):
			timestamp=curtime
			ard.acquire(',')
		if (curtime-printtime>0.1):
			printtime=curtime
			print str(ard.getData())

	run(setup,loop)
