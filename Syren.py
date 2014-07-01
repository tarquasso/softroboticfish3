#!/usr/bin/python
from bbio import *

class Syren:
	def __init__(self, port, baud):
		self.port=port
		self.port.begin(baud)

	def __del__(self):
		self.port.flush()
		self.port.end()
	
	def update(self, duty):
		control=float(duty)*(2.54)
		self.port.flush()
		self.port.prints(ord(control))
