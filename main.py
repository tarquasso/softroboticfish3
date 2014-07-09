#!/usr/bin/python
'''This is the main run file for the fish control system. run it'''

import os, sys, signal, curses
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
	window=curses.initscr()
	window.nodelay(1)
	window.addstr("running fish brainz!\n")
	window.addstr("press ctrl-c to kill")
	while (running==True):
		mainCtrl.control()
	curses.endwin()
	print "quitting"
	mainCtrl.cleanup()
	sys.exit(0)
