#!/usr/bin/python
from bbio import *
import os, sys, time

class Syren:
	def __init__(self, port, baud):
		self.port=port
		self.port.begin(baud)

	def finish(self):
		self.port.flush()
		self.port.end()
	
	def update(self, duty):
		control=float(duty)*(2.54)
		self.port.flush()
		self.port.prints(int(control))

if (__name__=="__main__"):
	bbio_init()
	sy=Syren(Serial1, 9600)
	sy.update(0)
	time.sleep(1)
	sy.update(50)
	time.sleep(1)
	sy.update(100)
	time.sleep(1)
	sy.update(50)
	sy.finish()
	bbio_cleanup()
