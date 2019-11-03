
import os,sys
import time
from bluetooth import (
    BluetoothSocket,
    RFCOMM,
)
from bluetooth.btcommon import BluetoothError


rfcomm = BluetoothSocket(RFCOMM)

rfcomm.connect(('00:18:E5:03:F5:99', 1)) # Bricogeek
#rfcomm.connect(('98:D3:31:F6:1C:51', 1)) # white
#rfcomm.connect(('98:D3:31:70:13:AB', 1)) # green

#rfcomm.settimeout(None) # set blocking True
#rfcomm.settimeout(0.0) # set blocking False
#rfcomm.settimeout(0.01)

print("Connected")

#t1 = time.time()
while (True):

    try:
        data = rfcomm.recv(1024)
        print(len(data))
    except BluetoothError as error:
        if str(error) != 'timed out':
            raise

    #try:
    #    rfcomm.send(data)
    #except BluetoothError as error:
    #    if str(error) != 'timed out':
    #        raise

    #t2 = time.time()
    #print(t2-t1)
    #t1 = time.time()


    #try:
    #    rfcomm.send(rfcomm.recv(1024))
    #    #data = rfcomm.recv(1024)
    #    #print(len(data))
    #except BluetoothError as error:
    #    if str(error) != 'timed out':
    #        raise

    #t2 = time.time()
    #print(t2-t1)
    #t1 = time.time()
