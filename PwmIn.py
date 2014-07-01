#!/usr/bin/python
#This class can read the pwm value on a GPIO pin
from bbio import *
import time #for timer functionality
import os, sys

class PwmIn:
	def __init__(self, pin, minDuty, maxDuty):
		self.pin=pin
		self.minDuty=minDuty
		self.maxDuty=maxDuty
		self.period=0
		self.pulsewidth=0
		self.risen=False
		pinMode(self.pin, INPUT)
		try:
			attachInterrupt(self.pin, self.rise, RISING)
			attachInterrupt(self.pin, self.fall, FALLING)
		except RuntimeError:
			detachInterrupt(self.pin)
			delay(4000)
			attachInterrupt(self.pin, self.rise, RISING)
			attachInterrupt(self.pin, self.fall, FALLING)
		self.time=time.clock() #start the timer

	def __del__(self):
		pass
		#cleanup
		detachInterrupt(self.pin)

	def getPeriod(self):
		return self.period

	def rise(self):
		_tmp=time.clock()
		diff=_tmp-self.time #look at time difference
		if (diff>=0.019): #rc receiver has 19ms period
			self.period=diff #know the period
			self.risen=True #we rose
			self.time=time.clock() #reset timer

	def fall(self):
		_tmp=time.clock()
		diff=_tmp-self.time
		if (diff>5e-4 and dif<0.005 and self.risen==True):
			self.risen=False
			self.pulsewidth=diff


#for debugging
if(__name__ == "__main__"):
	channel = PwmIn(GPIO1_28, 0, 100)
	pwmpin=PWM2B
	def setup():
		pwmWrite(pwmpin, 50)
		print "reading 100 times..."
		for i in xrange(100):
			print str(channel.getPeriod())
	def loop():
		sys.exit(0)
	run(setup,loop)

