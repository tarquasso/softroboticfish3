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
        self.temp=None
        self.press=None
        Wire2.begin()
        time.sleep(1)
        Wire2.quickwrite(IMUDepthSensor._devAddress, IMUDepthSensor._reset)
        time.sleep(1)
        for i in range(8):
            received = Wire2.readTransaction(IMUDepthSensor._devAddress, IMUDepthSensor._promBaseAddress+(2*i), 2)
            print 'received:',received
            if not isinstance(received,(list,tuple)) or len(received) is not 2:
                print 'Error: received less or more than two bytes'
            self.ByteHigh = received[0]
            self.ByteLow = received[1]
            self.CalConstant.append((self.ByteHigh << 8) + self.ByteLow)

        print 'Calibration constants are:'
        print self.CalConstant

    def getPressure(self):
        return self.press
    def getTemp(self):
        return self.temp
    def Reset(self):
        Wire2.quickwrite(IMUDepthSensor._devAddress, IMUDepthSensor._reset)
        time.sleep(1)
        return 0

    def ReadProm(self):
        for i in range(8):
            received = Wire2.readTransaction(IMUDepthSensor._devAddress, IMUDepthSensor._promBaseAddress+(2*i), 2)
            print 'received:',received
            if not isinstance(received,(list,tuple)) or len(received) is not 2:
                print 'Error: received less or more than two bytes'
            self.ByteHigh = received[0]
            self.ByteLow = received[1]
            self.CalConstant.append((self.ByteHigh << 8) + self.ByteLow)
        return 0
    def ConvertD1(self):
        "start the pressure sensor conversion"
        Wire2.quickwrite(IMUDepthSensor._devAddress, IMUDepthSensor._D1_512)
        return 0
    def ConvertD2(self):
        "start the temperature sensor conversion"
        Wire2.quickwrite(IMUDepthSensor._devAddress, IMUDepthSensor._D2_512)
        return 0
    def ReadADC(self):
        "return the conversion results from the last conversion request"
        results=Wire2.readTransaction(IMUDepthSensor._devAddress, IMUDepthSensor._adcRead, 3)
        try:
            return (results[2])+(results[1] << 8)+(results[0] << 16)
        except:
            return 0
    def compute(self):
        self.ConvertD1()
        time.sleep(0.5)
        D1=self.ReadADC()
        self.ConvertD2()
        time.sleep(0.5)
        D2=self.ReadADC()
        dT=D2-(self.CalConstant[5]*256)
        OFF=self.CalConstant[2]*(1<<16)+(dT*self.CalConstant[4])/(1<<7)
        SENS=self.CalConstant[1]*(1<<15)+(dT*self.CalConstant[3])/(1<<8)
        temp=2000+(dT*self.CalConstant[6])/(1<<23)
        self.temp=float(temp)/100.0
        press=((D1*SENS) / (1<<21) - OFF) / (1<<15)
        self.press=press
    def finish(self):
        Wire2.end()
#    def sendCommand(self, command):
#	print 'command:',command
#        numBytesWritten = Wire2.write(IMUDepthSensor._devAddress,0x00,command)
#        print 'Number of Bytes written:',numBytesWritten

if (__name__=="__main__"): # for debugging purposes when running just this file
	bbio.bbio_init()
	print 'Code is running'
        ids=IMUDepthSensor()
        print 'initialized'
	time.sleep(1)
        for i in xrange(100):
            ids.compute()
            print 'temp: '+ str(ids.getTemp()) + ', pressure: '+ str(ids.getPressure())
	ids.finish()
	bbio.bbio_cleanup()
# vim: tabstop=8 expandtab shiftwidth=4 softtabstop=4
