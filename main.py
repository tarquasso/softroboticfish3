#!/usr/bin/python
'''This is the main run file for the fish control system. run it'''

import os, sys, signal
import MainController

if (__name__=="__main__"):
	running=True
	def signal_handler(signal, frame):
		running=False
	
	signal.signal(signal.SIGINT, signal_handler)
	mainCtrl=MainController.MainController()
	while (running==True):
		mainCtrl.control()
	mainCtrl.cleanup()
	sys.exit(0)
