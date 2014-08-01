#!/usr/bin/python
#reads pwm input from the arduino via serial
from bbio import *
import os, sys

class Arduino:
	def __init__(self, serial, baud):
		self.serial=serial
		self.baudrate=baud
		self.serial.begin(self.baudrate)
	def __del__(self):
		self.serial.end()
	def acquire(self, delimeter):
		self.serial.printLn("GET")  #sends GET\n
		data=''
		#please format data as "<val><delimeter><val><delimeter>..."
		while (self.serial.available()):
			data.append(self.serial.read())
		return [float(i) for i in data.split(delimeter)]

if (__name__=="__main__"):
	ard=None
	def setup():
		global ard
		ard=Arduino(serial2, 9600)
	def loop():
		global ard
		print str(ard.acquire,',')
	run(setup,loop)
