#!/usr/bin/python
#This is the main controller class
import numpy as np #for camera
import select
import v4l2capture
import time
from bbio import *
import Syren, Fishgait, PwmIn, IMUDepthSensor, PID
#duty cycles will be 0-100

class MainController:
    ch1UpperBound=85.0
    ch1LowerBound=15.0
    ch1Duty=50.0
    ch2Duty=50.0
    ch3Duty=50.0
    ch4Duty=50.0
    def __init__(self):
        bbio.bbio_init()
        self.syren=Syren.Syren(Serial1, 19200) #init syren
        self.syren.update(50) #zero it for now
        self.leftservo=PWM1A    #left servo pwm out
        self.rightservo=PWM2A   #right servo pwm out
        self.sduservo=PWM2B     #servo for static diving unit
        self.ch1=PwmIn.PwmIn(GPIO3_19, 0.001, 0.002) #ch1 pwm in
        self.ch2=PwmIn.PwmIn(GPIO3_17, 0.001, 0.002) #..
        self.ch3=PwmIn.PwmIn(GPIO3_15, 0.001, 0.002) #..
        self.ch4=PwmIn.PwmIn(GPIO3_16, 0.001, 0.00195) #..
        #self.ids=IMUDepthSensor.IMUDepthSensor() #i2c depth sensor
        self.depth_controller=PID.PID()
        self.depth_controller.setPoint(20) #input the target depth
        pwmFrequency(self.leftservo, 100) #set the switching frequency of the servo outputs in Hz
        pwmFrequency(self.rightservo, 100)
        self.gait=Fishgait.SineGait(1.0, 1) #init a triangle-shaped fish gait with amplitude 0.5 and period 1s
        #init camera
        # Open the video device.
        self.video = v4l2capture.Video_device("/dev/video0")

        # Suggest an image size to the device. The device may choose and
        # rightservoeturn another size if it doesn't support the suggested one.
        self.size_x, self.size_y = self.video.set_format(1920, 1080, fourcc='H264')

        # Create a buffer to store image data in. This must be done before
        # calling 'start' if v4l2capture is compiled with libv4l2. Otherwise
        # raises IOError.
        self.video.create_buffers(30)
        # Send the buffer to the device. Some devices require this to be done
        # before calling 'start'.
        self.video.queue_all_buffers()

        self.video_record=False #true if we're currently recording
        self.f=None #the file to be saved
        self.video_trigger=True #controller hysteresis variable
        self.picture_trigger=True
        self.video.start()

    def cleanup(self):
        if (self.video_record):
            self.f.close()
            self.video.close()
        self.syren.finish()
        #self.ids.finish()
        bbio.bbio_cleanup()

    def getDutyCycles(self):
        MainController.ch1Duty=self.ch1.getDuty()
        MainController.ch2Duty=self.ch2.getDuty()
        MainController.ch3Duty=self.ch3.getDuty()
        MainController.ch4Duty=self.ch4.getDuty()

    def videocap(self):
        if (self.video_record == True):
            # Wait for the device to fill the buffer.
            select.select((self.video,), (), ())
            # The rest is easy :-)
            image_data = self.video.read_and_queue()
            self.f.write(image_data)

    def control(self):
        #self.ids.compute() #start the conversion process so that it's done by the time we use it
        duty=self.gait.compute() #get the current duty cycle
        self.syren.update(duty) #update syren
        #self.depth_controller.update(self.ids.getDepth())
        #analogWrite(self.leftservo, int(duty), RES_8BIT)
        #analogWrite(self.rightservo, int(duty), RES_8BIT)

    def take_picture(self):
        #self.size_x, self.size_y=self.video.set_format(1920,1080)
        with open("pic"+str(time.time())+".raw", 'wb') as f:
            select.select((self.video,),(),()) #wait to fill buffer
            image_data=self.video.read()
            f.write(image_data)

    def handle_rc_control(self):
        analogWrite(self.leftservo, MainController.ch3Duty, PERCENT)
        analogWrite(self.rightservo, MainController.ch3Duty, PERCENT)
        self.gait.update_yaw(MainController.ch4Duty)
        self.gait.update_freq(2*(MainController.ch2Duty/100.0))
        if (self.picture_trigger==False and MainController.ch1Duty<MainController.ch1LowerBound):
            self.picture_trigger=True
            self.take_picture()
            print 'took picture'
        elif (MainController.ch1Duty>MainController.ch1UpperBound):
            if (self.video_trigger==False):
                self.video_trigger=True
                if (self.video_record==True):
                    self.video_record=False
                    self.f.close()
        #           self.video.close()
                    print 'wrote video'
                elif (self.video_record==False):
                    self.video_record=True
       #            self.video.start()
                    self.f=open('video'+str(time.time())+'.raw', 'wb')
        elif (MainController.ch1Duty>MainController.ch1LowerBound and MainController.ch1Duty<MainController.ch1UpperBound):
            self.video_trigger=False
            self.picture_trigger=False
    def __str__(self):
        #return str(MainController.ch2Duty)
        return 'frq:%f amp:%f ch1:%i ch2:%i ch3:%i ch4:%i' % (self.gait.get_freq(), self.gait.get_amp(), MainController.ch1Duty, MainController.ch2Duty, MainController.ch3Duty, MainController.ch4Duty)
