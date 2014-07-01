#!/usr/bin/python
#This is the main controller class
from bbio import *
import Syren, Fishgait
#duty cycles will be 0-100

class MainController:
	def __init__(self):
		self.syren=Syren.Syren(Serial1, 9600) #init syren
		self.syren.update(50) #zero it for now
		self.leftservo=PWM2A
		self.rightservo=PWM2B
		self.gait=FishGait.SawGait(freq, amp)

	def control(self):
		duty=self.gait.compute() #get the current duty cycle
		self.syren.update(duty) #update syren
		pwmWrite(self.leftservo, 50)
		pwmWrite(self.rightservo, 50)
