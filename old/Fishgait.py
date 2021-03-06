#!/usr/bin/python
#fish gaits!
import time, os, sys, math

class fishgait:
    def __init__(self):
        self.timer=time.clock()
    def elapsedTime(self):
        return time.clock()-self.timer
    def resetTime(self):
        self.timer=time.clock()

class TriangleGait(fishgait):
    def __init__(self, freq, amplitude):
        fishgait.__init__(self) #init base class
        self.frq=float(freq)
        self.amp=float(amplitude)
        self.rising=True
    def get_freq(self):
        return self.frq
    def get_amp(self):
        return self.amp
    def update_freq(self, freq):
        if (float(freq)>=0.1):
            self.frq=float(freq)
    def update_amp(self, amplitude):
        if (float(amplitude)>=0.1 and float(amplitude)<=1):
            self.amp=float(amplitude)
    def compute(self):
        period=(1.0/self.frq) #period in sec
        curtime=self.elapsedTime() #read timer in sec
        if (curtime>period):
            self.rising=not self.rising
            self.resetTime()
        if (self.rising==True):
            out=curtime/period
        else:
            out=1.0-curtime/period
        out-=0.5 #center it around 0
        out*=self.amp #factor in amplitude
        out+=0.5 #recenter it around 0.5
        if (out>1):
            out=1
        elif (out<0):
            out=0
        return out*100

class SineGait2(fishgait):
    def __init__(self, freq, amplitude):
        fishgait.__init__(self) #init base class
        self.frq=float(freq)
        self.amp=float(amplitude)
        self.fullcycle=True
        self.yaw=0.0
    def get_freq(self):
        return self.frq
    def get_amp(self):
        return self.amp
    def update_freq(self, freq):
        if (float(freq)>=0.1):
            self.frq=float(freq)
    def update_amp(self, amplitude):
        if (float(amplitude)>=0.1 and float(amplitude)<=1):
            self.amp=float(amplitude)
    def update_yaw(self, yaw):
        self.yaw=2*((yaw/100.0)-0.5)
    def compute(self):
        period=(1.0/self.frq) #period in sec
        curtime=self.elapsedTime() #read timer in sec
        out=math.sin(2*math.pi*self.frq*curtime)+self.yaw
	#out=math.sin(2.0*math.pi*self.frq*curtime)
        out+=1 #center it around 1
        out*=0.5 #scale it
        if (out>1):
            out=1
        elif (out<0):
            out=0
        return out*100


class SineGait(fishgait):
    def __init__(self, freq, amplitude):
        fishgait.__init__(self) #init base class
        self.frq=float(freq)
        self.amp=float(amplitude)
        self.fullcycle=True
        self.yaw=0.6
    def get_freq(self):
        return self.frq
    def get_amp(self):
        return self.amp
    def update_freq(self, freq):
        if (float(freq)>=0.1):
            self.frq=float(freq)
    def update_amp(self, amplitude):
        if (float(amplitude)>=0.1 and float(amplitude)<=1):
            self.amp=float(amplitude)
    def update_yaw(self, yaw):
        self.yaw=2*((yaw/100.0)-0.5)
    def compute(self):
        period=(1.0/self.frq) #period in sec
        curtime=self.elapsedTime() #read timer in sec
        ampNew=self.amp
        if (curtime>(1/(2*self.frq))):
            if (self.fullcycle):
		if (self.yaw<0.0):
			ampNew=(1.0+0.7*self.yaw)*self.amp
		else:
			ampNew=self.amp
                self.fullcycle=False
            else:
                self.amp*=-1
		if (self.yaw>0.0):
			ampNew=(1.0-0.7*self.yaw)*self.amp
		else:
			ampNew=self.amp
                self.fullcycle=True
	    self.resetTime()
	    curtime=0.0
        out=ampNew*math.sin(2*math.pi*self.frq*curtime)
	#out=math.sin(2.0*math.pi*self.frq*curtime)
        out+=1 #center it around 1
        out*=0.5 #scale it
        if (out>1):
            out=1
        elif (out<0):
            out=0
        return out*100

class SawGait(fishgait):
    def __init__(self, freq, amplitude):
        fishgait.__init__(self) #init base class
        self.frq=float(freq)
        self.amp=float(amplitude)
    def get_freq(self):
        return self.frq
    def get_amp(self):
        return self.amp
    def update_freq(self, freq):
        if (float(freq)>=0.1):
            self.frq=float(freq)
    def update_amp(self, amplitude):
        if (float(amplitude)>=0.1 and float(amplitude)<=1):
            self.amp=float(amplitude)
    def compute(self):
        period=(1.0/self.frq) #period in sec
        curtime=self.elapsedTime() #read timer in sec
        if (curtime>period):
            self.resetTime()
        out=curtime/period
        out-=0.5 #center it around 0
        out*=self.amp #factor in amplitude
        out+=0.5 #recenter it around 0.5
        if (out>1):
            out=1
        elif (out<0):
            out=0
        return out*100


if (__name__=="__main__"):
    sgait=SawGait(3.0, 1.0)
    for i in xrange(10000):
        print str(sgait.compute())
