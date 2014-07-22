#!/usr/bin/python
#This is the main controller class
import numpy as np #for camera
import cv2 #for camera
import time
from bbio import *
import Syren, Fishgait
#duty cycles will be 0-100

class MainController:
	def __init__(self):
		bbio.bbio_init()
		self.syren=Syren.Syren(Serial1, 19200) #init syren
		self.syren.update(50) #zero it for now
		self.leftservo=PWM1A
		self.rightservo=PWM2A
		pwmFrequency(self.leftservo, 100)
		pwmFrequency(self.rightservo, 100)
		self.gait=Fishgait.TriangleGait(0.5, 1)
		#init camera
		self.cap=cv2.VideoCapture(0) #open default camera
		if (self.cap.isOpened()==False):
			self.cap=False #no camera :(
		self.cap.set(cv2.cv.CV_CAP_PROP_FRAME_WIDTH, 320)
		self.cap.set(cv2.cv.CV_CAP_PROP_FRAME_HEIGHT, 240)
		self.video=False #true if we're currently recording
	
	def cleanup(self):
		if (self.cap):
			self.cap.release()
		self.syren.finish()
		bbio.bbio_cleanup()

	def control(self):
		duty=self.gait.compute() #get the current duty cycle
		self.syren.update(duty) #update syren
		if (self.video != False):
			self.video.write(self.cap.read()[1])
		#analogWrite(self.leftservo, int(duty), RES_8BIT)
		#analogWrite(self.rightservo, int(duty), RES_8BIT)
	
	def handle_input(self, keycode):
		amount=0.01
		keycode=int(keycode) #for safety
		#keycode is a the ascii keycode of what was pressed
		if (keycode==ord('w')):
			self.gait.update_amp(self.gait.get_amp()+amount)
		elif (keycode==ord('s')):
			self.gait.update_amp(self.gait.get_amp()-amount)
		elif (keycode==ord('a')):
			self.gait.update_freq(self.gait.get_freq()+amount)	
		elif (keycode==ord('d')):
			self.gait.update_freq(self.gait.get_freq()-amount)
		elif (keycode==ord('p')):
			if (self.cap):
				cv2.imwrite("pic-"+str(time.time())+".png", self.cap.read()[1])
		elif (keycode==ord('v')):
			if (self.cap):
				if (self.video==False):
					self.video=cv2.VideoWriter('output'+str(time.time())+'.avi', cv2.cv.CV_FOURCC('X','2','6','4'), 20.0, (320,240))
					if (not self.video.isOpened()):
						self.video=False
				else:
					self.video=False
	def __str__(self):
		return 'frq:%f amp:%f' % (self.gait.get_freq(), self.gait.get_amp())
