#!/usr/bin/env python3
import serial
import time



if __name__ == '__main__':
    ser = serial.Serial('/dev/ttyACM0', 115200, timeout=1)
    ser.flush()
    
    ser.write(b"M114\n")
    line = ser.readline().decode('utf-8').rstrip()
    print(line)
    
    ser.write(b"M19\n")
    # line = ser.readline().decode('utf-8').rstrip()
    # print(line)

    ser.write(b"G11 I11 J11\n")
    # line = ser.readline().decode('utf-8').rstrip()
    # print(line)
    time.sleep(2)

    ser.write(b"M18\n")
    # line = ser.readline().decode('utf-8').rstrip()
    # print(line)
