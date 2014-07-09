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
	window.addstr(str(os.uname())+"\n")
	window.addstr("running fish brainz! ..send sigint to quit\n")
	start=time.clock()
	while (running==True):
		mainCtrl.handle_input(window.getch())
		mainCtrl.control()
		window.addstr(2,0,'uptime: '+str(time.clock()-start))
		window.addstr(3,0,str(mainCtrl))
	window.addstr(4,0,"quitting")
	time.sleep(1.0)
	curses.endwin()
	mainCtrl.cleanup()
	print "exit success"
	sys.exit(0)
