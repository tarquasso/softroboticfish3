from bbio import *

Wire2.end()
Wire2=None #first kill the stupid i^2c bus

class Betteri2c(_I2C_BUS): #inherit from the bad class to change 1 function
    def __init__(self, bus):
        _I2C_BUS.__init__(self, bus)
    def write_bytes(self, addr, data, length):
        if (not self.open):
            print 'I2C bus not initialized'
            return
        try:
            if (length==1):
                self.bus.write_byte(addr,data)
            else:
                self.bus.write_i2c_block_data(addr, data[0], data[1::])
        except IOError as e:
            print 'Bus is active, try again?'
    def read_bytes(self, addr, length):
        output=[]
        if (not self.open):
            print 'I2C bus not initialized'
            return
        try:
            if (length==1):
                output.append(self.bus.read_byte(addr))
            else:
                output.append(self.bus.read_i2c_block_data(addr
        except IOError as e:
            print 'Bus is active, try again?'
