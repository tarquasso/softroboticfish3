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
# vim: tabstop=8 expandtab shiftwidth=4 softtabstop=4
