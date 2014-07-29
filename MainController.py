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
    def __init__(self):
        bbio.bbio_init()
        self.syren=Syren.Syren(Serial1, 19200) #init syren
        self.syren.update(50) #zero it for now
        self.leftservo=PWM1A    #left servo pwm out
        self.rightservo=PWM2A   #right servo pwm out
        self.sduservo=PWM2B     #servo for static diving unit
        self.ch1=PwmIn.PwmIn(GPIO3_19, 0.00, 0.002) #ch1 pwm in
        self.ch2=PwmIn.PwmIn(GPIO3_17, 0.00, 0.002) #..
        self.ch3=PwmIn.PwmIn(GPIO3_15, 0.00, 0.002) #..
        self.ch4=PwmIn.PwmIn(GPIO3_16, 0.00, 0.002) #..
        #self.ids=IMUDepthSensor.IMUDepthSensor() #i2c depth sensor
        self.depth_controller=PID.PID()
        self.depth_controller.setPoint(20) #input the target depth
        pwmFrequency(self.leftservo, 100) #set the switching frequency of the servo outputs in Hz
        pwmFrequency(self.rightservo, 100)
        self.gait=Fishgait.TriangleGait(0.5, 1) #init a triangle-shaped fish gait with amplitude 0.5 and period 1s
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
        self.video.start()

    def cleanup(self):
        if (self.video_record):
            self.f.close()
            self.video.close()
        self.syren.finish()
        #self.ids.finish()
        bbio.bbio_cleanup()

    def control(self):
        #self.ids.compute() #start the conversion process so that it's done by the time we use it
        duty=self.gait.compute() #get the current duty cycle
        self.syren.update(duty) #update syren
        #self.depth_controller.update(self.ids.getDepth())
        if (self.video_record == True):
            # Wait for the device to fill the buffer.
            select.select((self.video,), (), ())
            # The rest is easy :-)
            image_data = self.video.read_and_queue()
            self.f.write(image_data)
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
            if (self.video_record==True):
                self.video_record=False
                self.f.close()
        #        self.video.close()
                print 'wrote video'
            elif (self.video_record==False):
                self.video_record=True
       #         self.video.start()
                self.f=open('video'+str(time.time())+'.raw', 'wb')
    def __str__(self):
        return 'frq:%f amp:%f' % (self.gait.get_freq(), self.gait.get_amp())
