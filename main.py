#!/usr/bin/python
'''This is the main run file for the fish control system. run it'''

import os, sys, signal, curses
import MainController
from bbio import *

if (__name__=="__main__"):
	running=True
	def signal_handler(signal, frame):
		global running
		running=False
	
	window=curses.initscr()
	window.nodelay(1)
	signal.signal(signal.SIGINT, signal_handler)
	mainCtrl=MainController.MainController()
	window.addstr("running fish brainz!\n")
	while (running==True):
		ch=window.getch()
		if (int(ch)>=0):
			window.addstr(str(ch))
		mainCtrl.control()
	curses.endwin()
	print "quitting"
	mainCtrl.cleanup()
	sys.exit(0)
