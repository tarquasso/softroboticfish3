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

    #window=curses.initscr()
    #window.nodelay(1)
    signal.signal(signal.SIGINT, signal_handler)
    mainCtrl=MainController.MainController()
    #window.addstr(str(os.uname())+"\n")
    #window.addstr("running fish brainz! ..send sigint to quit\n")
    start=time.clock()
    printclock=time.clock()
    while (running==True):
     #   window.getch()
        #mainCtrl.handle_rc_control()
     #   print str(mainCtrl)
        mainCtrl.videocap()
        if ((time.clock()-start)>0.005):
            #window.addstr(2,0,'uptime: '+str(time.clock()-start))
            #window.addstr(3,0,str(mainCtrl))
            mainCtrl.getDutyCycles()
            mainCtrl.control()
            mainCtrl.handle_rc_control()
            start=time.clock()
        if ((time.clock()-printclock)>1.0):
            print str(mainCtrl)
            printclock=time.clock()
        #time.sleep(1.0)
    #window.addstr(4,0,"quitting")
    time.sleep(1.0)
    #curses.endwin()
    mainCtrl.cleanup()
    print "exit success"
    sys.exit(0)
