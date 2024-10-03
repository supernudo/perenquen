
import os,sys
import time
from bluetooth import (
    BluetoothSocket,
    RFCOMM,
)
from bluetooth.btcommon import BluetoothError

#rfcomm = BluetoothSocket(RFCOMM)
#rfcomm.connect(('98:D3:31:70:13:AB', 1))
#rfcomm.settimeout(0.01)

def tm_enable():
    rfcomm.settimeout(1.)
    rfcomm.send("\n\revent tm on\n\r")

def tm_read():
    #rfcomm.settimeout(None) # set blocking True
    #rfcomm.settimeout(0.0) # set blocking False
    #rfcomm.settimeout(1.)
    #time.sleep(5)
    t1 = time.time()
    t2 = time.time()
    while time.time() - t1 < 60:
        try:
            rfcomm.settimeout(None)
            received = rfcomm.recv(2**16)
            print(time.time() -t2, len(received))
            t2 = time.time()
            #time.sleep(0.001)
        except BluetoothError as error:
            if str(error) == 'timed out':
                print('time out')
            else:
                raise
                return


if __name__ == '__main__':
    #tm_enable()
    #time.sleep(5)
    #tm_read()

    #rfcomm.close()

    rfcomm = BluetoothSocket(RFCOMM)
    #rfcomm.connect(('98:D3:31:F6:1C:51', 1)) # white
    rfcomm.connect(('98:D3:31:70:13:AB', 1)) # green
    rfcomm.settimeout(0.01)

    while (True):
        data = rfcomm.recv(2**16)
        print(data)
