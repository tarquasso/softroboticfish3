#!/usr/bin/python
#This is the MainController class
import os
os.chdir('/home/pi/fishbrainz/rpi')
import sys,time,Camera,MbedSerial


class MainController:
	def __init__(self):
		self.camera=Camera.FishCamera()
		self.mbed=MbedSerial.MbedSerial("/dev/ttyAMA0", 9600)
		self.takingvideo=False
	def control(self):
		command=self.mbed.getbytes(1)
		print command
		
		if (command == 'c' and self.takingvideo==False):
			self.camera.take_picture()
			print 'took picture'
		elif (command == 'a'):
			self.takingvideo=True
			self.camera.take_video()
			print 'start video'
		elif (command == 'b'):
			self.takingvideo=False
			self.camera.stop_video()
			print 'stop video'
		else:
			print 'neutral position'

if (__name__=="__main__"):
	mCtrl=MainController()
	while 1:
		mCtrl.control()
