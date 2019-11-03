
import serial
import time

ser = serial.Serial('/dev/ttyUSB0',baudrate=921600)
#ser = serial.Serial('/dev/rfcomm4',baudrate=921600, xonxoff=True)
#ser = serial.Serial('COM10',baudrate=921600)
while (True):
    data = ser.readline()
    print(data)
