#!/usr/bin/python
from bbio import *
import PwmInArduino
import os,sys,time

if (__name__=="__main__"):
	ard=None
	servo=None
	timestamp=time.time()
	printtime=time.time()
	data=[0,0,0,0]
	duty=0.0
	sigrange=1900-1100
	def setup():
		global ard
		global servo
		ard=PwmInArduino.Arduino(Serial2,9600)
		servo=PWM1A
		pwmFrequency(PWM1A, 50)
		pass
	def loop():
		global servo
		global ard
		global timestamp
		global printtime
		global data
		global duty
		curtime=time.time()
		duty=int((2**16)*((data[3])/23000.0))
		analogWrite(servo, duty, RES_16BIT)
		#if (curtime-timestamp>0.001):
		#	timestamp=curtime
		ard.acquire(',')
		if (curtime-printtime>0.1):
			printtime=curtime
			data=ard.getData()
			print "data:%s duty:%f" % (str(data), data[3])
	run(setup,loop)
