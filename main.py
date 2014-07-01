#!/usr/bin/python
'''This is the main run file for the fish control system. run it'''

import os, sys, signal
import MainController
from bbio import *

if (__name__=="__main__"):
	running=True
	def signal_handler(signal, frame):
		print "got sigint"
		global running
		running=False
	
	signal.signal(signal.SIGINT, signal_handler)
	mainCtrl=MainController.MainController()
	print "running. press ctrl-c to kill"
	while (running==True):
		mainCtrl.control()
	print "quitting"
	mainCtrl.cleanup()
	sys.exit(0)
