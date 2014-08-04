import spidev
import time
spi = spidev.SpiDev()
spi.open(0, 1)
try:
    while True:
        resp = spi.xfer2([0xAA])
        print resp[0]
        time.sleep(1)
    #end while
except KeyboardInterrupt:
    # create spi object
    # open spi port 0, device (CS) 1
    # transfer one byte
    # sleep for 0.1 seconds
    # Ctrl+C pressed, so...
    spi.close() # ... close the port before exit #end try
#This script will reverse the bit ordering in one byte (if you are not able to change LSB / MSB first to your needs.
def ReverseBits(byte):
    byte = ((byte & 0xF0) >> 4) | ((byte & 0x0F) << 4) 
    byte = ((byte & 0xCC) >> 2) | ((byte & 0x33) << 2) 
    byte = ((byte & 0xAA) >> 1) | ((byte & 0x55) << 1) 
    return byte
#end def

#Print bytes - This script will print out a byte array in a human readable format (hexadecimal). 
# This is often useful during debugging.
def BytesToHex(Bytes):
    return ''.join(["0x%02X " % x for x in Bytes]).strip()
#end def

# vim: tabstop=8 expandtab shiftwidth=4 softtabstop=4
