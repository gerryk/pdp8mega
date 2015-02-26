import serial
from time import sleep
import sys

ser = serial.Serial('/dev/ttyACM0', 9600)
bytes_read = open("lunar.txt", "rb").read()
for b in bytes_read:
    if (ord(b)!=10):
        ser.write(b)
        print b,ord(b)
        sleep(0.12)
