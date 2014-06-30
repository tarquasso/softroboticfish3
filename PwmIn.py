#!/usr/bin/python
#This class can read the pwm value on a GPIO pin
import Adafruit_BBIO.GPIO as GPIO
import time #for timer functionality
import threading, os, sys

class PwmIn:
	def __init__(self, pin, minDuty, maxDuty):
		self.pin=pin
		self.minDuty=minDuty
		self.maxDuty=maxDuty
		self.period=0
		self.pulsewidth=0
		self.risen=False
		GPIO.setup(self.pin, GPIO.IN)
		self.trise=threading.Thread(target=self.rise)
		self.tfall=threading.Thread(target=self.fall)
		self.time=time.clock() #start the timer
		self.trise.start()
		self.fall.start()

	def __del__(self):
		#cleanup
		self.trise.join()
		self.tfall.join()

	def getPeriod(self):
		return self.period

	def rise(self):
		GPIO.wait_for_edge(self.pin, GPIO.RISING) #wait for the rising edge
		_tmp=time.clock()
		diff=_tmp-self.time #look at time difference
		if (diff>=0.019): #rc receiver has 19ms period
			self.period=diff #know the period
			self.risen=True #we rose
			self.time=time.clock() #reset timer

	def fall(self):
		GPIO.wait_for_edge(self.pin, GPIO.FALLING)
		_tmp=time.clock()
		diff=_tmp-self.time()
		if (diff>5e-4 and dif<0.005 and self.risen==True):
			self.risen=False
			self.pulsewidth=diff


#for debugging
if(__name__ == "__main__"):
	PwmIn channel("P8_14", 0, 100)
	print "reading 100 times..."
	for i in xrange(100):
		print str(channel.getPeriod())

