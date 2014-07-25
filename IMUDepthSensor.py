#!/usr/bin/python
from bbio import *
import os, sys, time

class IMUDepthSensor:
    _devAddress = 0x76 # 7-bit I2C address of the MS5803
    _reset=0x1E
    _D1_256 = 0x40
    _D1_512 = 0x42
    _D1_1024 = 0x44
    _D1_2048 = 0x46
    _D1_4096 = 0x48
    _D2_256 = 0x50
    _D2_512 = 0x52
    _D2_1024 = 0x54
    _D2_2048 = 0x56
    _D2_4096 = 0x58
    _adcRead = 0x00
    _promBaseAddress = 0xA0
    def __init__(self):
        self.ByteHigh = 0
        self.ByteLow = 0
        self.CalConstant = []
	Wire2.begin()
        time.sleep(1)
        self.sendCommand(IMUDepthSensor._reset)
        time.sleep(1)
        for i in range(8):
            self.sendCommand(IMUDepthSensor._promBaseAddress + (2*i))
            received = Wire2.read(IMUDepthSensor._devAddress,0x00,2)
            print 'received:',received
            if not isinstance(received,(list,tuple)) or len(received) is not 2:
                print 'Error: received less or more than two bytes'
            
            self.ByteHigh = received[0]
            self.ByteLow = received[1]
            self.CalConstant.append((self.ByteHigh << 8) + self.ByteLow)

        print 'Calibration constants are:'
        print self.CalConstant

    def finish(self):
	Wire2.end()
	
    def sendCommand(self, command):
	print 'command:',command
        numBytesWritten = Wire2.write(IMUDepthSensor._devAddress,0x00,command)
        print 'Number of Bytes written:',numBytesWritten

if (__name__=="__main__"): # for debugging purposes when running just this file
	bbio.bbio_init()
	print 'Code is running'
        ids=IMUDepthSensor()
        print 'initialized'
	time.sleep(1)
	ids.finish()
	bbio.bbio_cleanup()
# vim: tabstop=8 expandtab shiftwidth=4 softtabstop=4
