#!/usr/bin/python
#This is the main controller class
from bbio import *
import Syren, Fishgait
#duty cycles will be 0-100

class MainController:
	def __init__(self):
		bbio_init()
		self.syren=Syren.Syren(Serial2, 19200) #init syren
		self.syren.update(50) #zero it for now
		self.leftservo=PWM1A
		self.rightservo=PWM2A
		pwmFrequency(self.leftservo, 100)
		pwmFrequency(self.rightservo, 100)
		self.gait=Fishgait.SawGait(0.5, 1)
	
	def cleanup(self):
		self.syren.finish()
		bbio_cleanup()

	def control(self):
		duty=self.gait.compute() #get the current duty cycle
#		self.syren.update(duty) #update syren
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
	def __str__(self):
		return 'frq:%f amp:%f' % (self.gait.get_freq(), self.gait.get_amp())
