#!/usr/bin/python
#This is the camera class
import picamera
import spidev
import time

class MbedSpi:
    do_nothing=int(0x00)
    take_picture=int(0x0A)
    start_video=int(0xAB)
    stop_video=int(0xEA)
    def __init__(self):
        #init spi
        self.spi = spidev.SpiDev()   # create spi object
        self.spi.open(0, 0)          # open spi port 0, device CE0 (CS 0)
        self.spi.mode=0
        self.command=0
    
    def __enter__(self):
        return self

    def command_response(self):
        #please make sure command is a byte
        resp=self.spi.xfer2([self.command])
        self.command=resp[0]
        return self.command
    
    def __str__(self):
        return 'Spi interface'

    def cleanup(self):
        self.spi.close()

    def __exit__(self,type,value,traceback):
        self.cleanup()
if (__name__=="__main__"): # for debugging purposes when running just this file
	print 'Code is running'
        with MbedSpi() as spi:
            print 'initialized'
            while (1):
                print spi.command_response()
                time.sleep(0.25)

# vim: tabstop=8 expandtab shiftwidth=4 softtabstop=4
