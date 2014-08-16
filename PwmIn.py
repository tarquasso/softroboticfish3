#!/usr/bin/python
#This class can read the pwm value on a GPIO pin
from bbio import *
import time #for timer functionality
import os, sys

#min is 1.12ms
#max is 2ms

class PwmIn:
	def __init__(self, pin, minPulse=0.001, maxPulse=0.002):
		self.pin=pin
		self.minPulse=minPulse
		self.maxPulse=maxPulse
		self.Rl=0
		self.Rh=0
		pinMode(self.pin, INPUT)
		try:
			attachInterrupt(self.pin, self.edge, BOTH)
		except RuntimeError:
			detachInterrupt(self.pin)
			delay(4000)
			attachInterrupt(self.pin, self.edge, BOTH)
		self.time=time.time() #start the timer

	def __del__(self):
		pass
		#cleanup
		detachInterrupt(self.pin)

	def getPulse(self):
		return self.Rh

        def getDuty(self):
            prange=self.maxPulse-self.minPulse
            return 100.0*((self.Rh-prange)/(prange))

	def edge(self):
		#we only know its an edge so we must orient ourselves
		dif=time.time()-self.time
		if (dif<=self.maxPulse and dif>self.minPulse):
			#this was the high pulse. Rh
			self.Rh=dif
		else:
			#this was the remainder of the pulse. Rl
			self.Rl=dif
			self.time=time.time() #reset the timer

#for debugging
if(__name__ == "__main__"):
	channel = PwmIn(GPIO1_28, 0.001, 0.002)
	count=0
	def setup():
		print "reading..."
	def loop():
		global count
		pass
		if (count>=100000):
			#print str(channel.times)
                        print 'Rh:'+str(channel.Rh)+', Rl:'+str(channel.Rl) +', PW:'+str(channel.Rl+channel.Rh) +', DUTY:' + str(channel.getDuty())
			count=0
			#sys.exit(0)
		count+=1
	#	sys.exit(0)
	run(setup,loop)
# vim: tabstop=8 expandtab shiftwidth=4 softtabstop=4
