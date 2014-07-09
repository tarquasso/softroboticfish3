#!/usr/bin/python
'''This is the main run file for the fish control system. run it'''

import os, sys, signal, curses, time
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
	window.addstr("running fish brainz! ..send sigint to quit\n")
	start=time.clock()
	while (running==True):
		mainCtrl.handle_input(window.getch())
		mainCtrl.control()
		window.addstr(1,0,'uptime: '+str(time.clock()-start))
		window.addstr(2,0,str(mainCtrl))
	curses.endwin()
	print "quitting"
	mainCtrl.cleanup()
	sys.exit(0)
