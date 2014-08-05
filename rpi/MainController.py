#!/usr/bin/python
#This is the MainController class
import os,sys,time,Camera,MbedSPI


class MainController:
	def __init__(self):
		self.camera=Camera.FishCamera()
		self.spi=MbedSPI.MbedSpi()
	def control(self):
		command=self.spi.command_response(0xAA)
		print command
		
		if (command == 10):
			self.camera.take_picture()
			print 'took picture'
		elif (command == 171):
			self.camera.take_video()
			print 'start video'
		elif (command == 234):
			self.camera.stop_video()
			print 'stop video'

if (__name__=="__main__"):
	mCtrl=MainController()
	timestamp=time.time()
	while ((time.time()-timestamp)<=20.0):
		mCtrl.control()
		time.sleep(0.25)
