#!/usr/bin/python
'''This is the main run file for the fish control system. run it'''

import os, sys
import MainController

if (__name__=="__main__"):
	mainCtrl=MainController.MainController()
	while 1:
		mainCtrl.control()
