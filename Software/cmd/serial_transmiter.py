
import serial
import time

ser = serial.Serial('/dev/ttyUSB0',baudrate=921600)
#ser = serial.Serial('COM12',baudrate=921600)
data = 'a'*19
data += '\n'
period = 0.0008
print('len(data', len(data)+1, 'period', period, 'bytes/ms', (len(data))/0.001/1000.0)
while (True):
    ser.write(data.encode('utf-8'))
    while(ser.out_waiting > 0):
        ser.out_waiting
        #print(ser.out_waiting)
    time.sleep(period)
