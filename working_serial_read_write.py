#!/usr/bin/env python3

import serial
import time

pos = [164.5, 100.0, 141.0, 90.0, 90.0, 0.0]

if __name__ == '__main__':
    ser = serial.Serial('COM3', 115200, timeout=.1)
    ser.flushInput()
    ser.flushOutput()
    ser.reset_input_buffer()
    # while True:
   
    while True:
        # if (ser.inWaiting() > 0):
        try:
            line = ser.readline()
            if line:
                string = line.decode('utf-8').rstrip()
                print(string)
                # splitPacket = string.split('\t')
                # for i in range(6):
                #    print(float(splitPacket[i]))
        except UnicodeDecodeError as e:
            print(e)
    time.sleep(1)
    ser.close()
