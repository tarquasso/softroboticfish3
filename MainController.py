#!/usr/bin/python
#This is the main controller class
import numpy as np #for camera
import cv2 #for camera
import time
from bbio import *
import Syren, Fishgait, PwmIn, IMUDepthSensor, PID
#duty cycles will be 0-100

class MainController:
    def __init__(self):
        bbio.bbio_init()
        self.syren=Syren.Syren(Serial1, 19200) #init syren
        self.syren.update(50) #zero it for now
        self.leftservo=PWM1A    #left servo pwm out
        self.rightservo=PWM2A   #right servo pwm out
        self.sduservo=PWM2B     #servo for static diving unit
        self.ch1=PwmIn.PwmIn(GPIO3_19, 0.00, 0.002) #ch1 pwm in
        self.ch2=PwmIn.PwmIn(GPIO3_17, 0.00, 0.002) #..
        self.ch3=PwmIn.PwmIn(GPIO3_29, 0.00, 0.002) #..
        self.ch4=PwmIn.PwmIn(GPIO3_16, 0.00, 0.002) #..
        self.ids=IMUDepthSensor.IMUDepthSensor() #i2c depth sensor
        self.depth_controller=PID.PID()
        self.depth_controller.setPoint(20) #input the target depth
        pwmFrequency(self.leftservo, 100) #set the switching frequency of the servo outputs in Hz
        pwmFrequency(self.rightservo, 100)
        self.gait=Fishgait.TriangleGait(0.5, 1) #init a triangle-shaped fish gait with amplitude 0.5 and period 1s
        #init camera
        self.cap=cv2.VideoCapture(0) #open default camera
        if (self.cap.isOpened()==False):
            self.cap=False #no camera :(
        self.width=640 #higher resolutions cause lag
        self.height=480
        self.cap.set(cv2.cv.CV_CAP_PROP_FRAME_WIDTH, self.width)
        self.cap.set(cv2.cv.CV_CAP_PROP_FRAME_HEIGHT, self.height)
        self.video=False #true if we're currently recording

    def cleanup(self):
        if (self.cap):
            self.cap.release()
        self.syren.finish()
        self.ids.finish()
        bbio.bbio_cleanup()

    def control(self):
        self.ids.compute() #start the conversion process so that it's done by the time we use it
        duty=self.gait.compute() #get the current duty cycle
        self.syren.update(duty) #update syren
        self.depth_controller.update(self.ids.getDepth())
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
                    self.video=cv2.VideoWriter('output'+str(time.time())+'.avi', cv2.cv.CV_FOURCC('M','J','P','G'), 20.0, (self.width,self.height))
                    if (not self.video.isOpened()):
                        self.video=False
                else:
                    self.video=False
    def __str__(self):
        return 'frq:%f amp:%f' % (self.gait.get_freq(), self.gait.get_amp())
