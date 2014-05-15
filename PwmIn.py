#!/usr/bin/python
#This class can read the pwm value on a GPIO pin
from bbio import *
import cTimer as timer #for timer functionality
import time
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
                self.last4=[0,0,0,0]
		self.width=0
		self.pulsewidth=20
		self.period=0.02
		self.inmax=0.0015
		self.inmin=0.0015
		pinMode(self.pin, INPUT)
		try:
			attachInterrupt(self.pin, self.edge, BOTH)
		except RuntimeError:
			detachInterrupt(self.pin)
			delay(4000)
			attachInterrupt(self.pin, self.edge, BOTH)
		self.started=False
		self.risen=False
		self.reading=None
		self.state=None
		self.tmp=None
		self.starttime=time.time()

	def __del__(self):
		pass
		#cleanup
		detachInterrupt(self.pin)

        def rotate(self, l, n):
            return l[n:]+l[:n]

	def getPulse(self):
		return self.Rh

	def getDutyAdjusted(self):
		duty=self.pulsewidth/self.period
		if (duty>.029):
			out=(duty-.029)/(0.054-0.029)
		else:
			out=0
		if (out<1.0):
			return out
		else:
			return 1.0
        def getDuty(self):
            prange=self.maxPulse-self.minPulse
            avg=sum(self.last4)/len(self.last4)
            #return 100.0*((self.Rh-prange)/prange)
	    return 100.0*((avg-prange)/(prange))
	def high(self):
		self.width=time.time()-self.time
		self.time=time.time()
	def edge(self):
		#we only know its an edge so we must orient ourselves
		#state=digitalRead(self.pin)
		if (self.started==False):
			self.start=timer.start()
			self.started=True
		else:
			self.stop=timer.stop()
			self.started=False
			dif=timer.diff(self.start, self.stop)
			if (dif<=self.maxPulse and dif>self.minPulse):
				#this was the high pulse. Rh
				self.Rh=dif
				self.last4=self.rotate(self.last4,-1)
				self.last4[3]=self.Rh
			if (dif>self.maxPulse):
				#were at the wrong edge
				self.start=timer.start()
				self.started=True
                        #self.last4=self.rotate(self.last4,-1)
                        #self.last4[3]=self.Rh
		#else:
			#this was the remainder of the pulse. Rl
		#	self.Rl=dif
		#	self.time=time.time() #reset the timer
		#self.pulsewidth=self.Rh+self.Rl

#for debugging
if(__name__ == "__main__"):
        ch1=PwmIn(GPIO3_19, 0.001, 0.002) #ch1 pwm in
        ch2=PwmIn(GPIO3_17, 0.001, 0.002) #..
        ch3=PwmIn(GPIO3_15, 0.001, 0.002) #..
        ch4=PwmIn(GPIO3_16, 0.001, 0.002) #..
	count=0
	dutymin=4.2
	dutymax=4.2
	def setup():
		print "reading..."
	def loop():
		global count
		global dutymin
		global dutymax
		pass
		if (count>=100000):
			#print strchannel.times)
                        #print 'Rh:'+str(channel.Rh)+', Rl:'+str(channel.Rl) +', PW:'+str(channel.Rl+channel.Rh) +', DUTY:' + str(channel.getDuty()) + " " +str(ch2.getDuty())
			#print 'pulsewidth %f' % (ch1.width)
			#print 'max %f min %f' % (ch1.inmax, ch1.inmin)
			if (ch3.getDuty()>dutymax):
				dutymax=ch3.getDuty()
			elif (ch3.getDuty()<dutymin):
				dutymin=ch3.getDuty()
			#print 'max %f min %f' % (dutymax, dutymin)
			#print str(ch3.getDuty())
			print 'ch1 ' +str(ch1.getDuty()) + ', ch2 ' + str(ch2.getDuty()) + ', ch3 ' + str(ch3.getDuty()) + ', ch4 ' + str(ch4.getDuty())
                        count=0
			#sys.exit(0)
		count+=1
	#	sys.exit(0)
	run(setup,loop)
# vim: tabstop=8 expandtab shiftwidth=4 softtabstop=4
