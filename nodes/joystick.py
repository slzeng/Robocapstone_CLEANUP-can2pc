# -*- coding: utf-8 -*-

import serial
import pygame
from time import sleep
from random import random
import ctypes

joystick = pygame.joystick

# Axis 0 : left -> right
# Axis 1:  front -> back
# Axis 2:  top -> bottom (scroll wheel)
# Buttons: NUM - 1, 0 is trigger

welcome = """
______  __                        _________      _____
___  / / /_________  _______________  ____/_____ __  /_
__  /_/ /_  __ \  / / /_  ___/  _ \  /    _  __ `/  __/
_  __  / / /_/ / /_/ /_(__  )/  __/ /___  / /_/ // /_
/_/ /_/  \____/\__,_/ /____/ \___/\____/  \__,_/ \__/
                              B
                        |-----------|
Red   : Left            |    9v     |
                        |           |
Blue  : Front           |           |
                       R| C       O |Y
Yellow: Right           |           |
                        |           |
Purple: Back            |electronics|
                        |-----------|
                              P
"""


def cap(num, lo, hi) :
    return min(max(num, lo), hi)

def toByte(real):
    return min(int((real + 1) * 128), 255)

def filter(real):
    if abs(real) < 0.1:
        return 0
    return real

def writeBytes(L,ser):
    for c in L:
        ser.write(chr(c))
        # print (chr(c).__repr__())
def readBytesStr(num,ser):
    return ser.read(num)
    
def readBytesRaw(num,ser):
    raw_string = ser.read(num)
    raw_bytes = len(raw_string)*[0]
    for i in range(0,len(raw_string)):
        raw_bytes[i] = ord(raw_string[i])
    return raw_bytes

def calcCRC(byteArray):
    crc = 0
    for i in range(0,len(byteArray)):
        crc = crc ^ (byteArray[i]<<8)
        for j in range(0,8):
            if(crc & 0x8000):
                crc = ((crc << 1) & 0xFFFF) ^ 0x1021
            else:
                crc = (crc << 1) & 0xFFFF
    return crc

def getPcktMotorPwrM1(velocity):
    if(velocity >= 0):
        assert(0 <= velocity and velocity <= 127)
        pckt = [0x80,0,velocity]
        crc = calcCRC(pckt)
        return pckt+[(crc>>8)&0xFF,crc&0xFF]
    else:
        assert(-127 <= velocity and velocity <= 0)
        pckt = [0x80,1,abs(velocity)]
        crc = calcCRC(pckt)
    return pckt+[(crc>>8)&0xFF,crc&0xFF]

def getPcktMotorPwrM2(velocity):
    if(velocity >= 0):
        assert(0 <= velocity and velocity <= 127)
        pckt = [0x80,4,velocity]
        crc = calcCRC(pckt)
        return pckt+[(crc>>8)&0xFF,crc&0xFF]
    else:
        assert(-127 <= velocity and velocity <= 0)
        pckt = [0x80,5,abs(velocity)]
        crc = calcCRC(pckt)
    return pckt+[(crc>>8)&0xFF,crc&0xFF]

def main():
    pygame.init()
    print "%d joystick(s) found" % joystick.get_count()
    if joystick.get_count() == 0 : return
    stick = joystick.Joystick(0)
    stick.init()

    print "[Joystick] Connected to %s Joystick" % stick.get_name()
    print "[Joystick] %d axes" % stick.get_numaxes()
    print "[Joystick] %d balls" % stick.get_numballs()
    print "[Joystick] %d buttons" % stick.get_numbuttons()
    # print "[Joystick] %d hats" % stick.get_numhats()

    ser = serial.Serial("/dev/ttyUSB0",115200,timeout=0)
    ser.flushInput()
    ser.flushOutput()
    #Check Roboclaw firmware version
    writeBytes([0x80,21],ser)
    sleep(0.1)
    temp = readBytesStr(30,ser)
    if(temp[0:26] == 'USB Roboclaw 2x60a v4.1.13'):
        print "[Serial] Connected to Roboclaw via %s" % ser.name
        print temp[0:26]
    else:
        print "Unable to connect to Roboclaw on %s" % ser.name
    #Check battery
    writeBytes([0x80,24],ser)
    sleep(0.1)
    temp = readBytesRaw(4,ser)
    batt_raw = (temp[0]<<8)|temp[1]
    print "Battery Voltage: %f" % (float(batt_raw)/10)
    x = calcCRC([0x80,24]+temp[0:2])
    assert(x == (temp[2]<<8)|temp[3]) #Make sure CRC algorithm is correct
    while True:
    #Get joystick state
        pygame.event.pump()
        axis_x = toByte(stick.get_axis(0))-128
        axis_y = -(toByte(stick.get_axis(1))-128)
        axis_z = -(toByte(stick.get_axis(2))-128)
        #Mixing
        left = cap(axis_x+axis_y,-127,127)
        right = cap(-axis_x+axis_y,-127,127)
        scale = float(128+cap(axis_z,-128,127))/255
        #print str(axis_x)+" "+str(axis_y)+" "+str(axis_z)
        #print str(left)+" "+str(right)
        
        #Send values to motors
        pckt = getPcktMotorPwrM1(int(left*scale))
        writeBytes(pckt,ser)
        sleep(0.05)
        temp = readBytesRaw(1,ser)
        if(temp != 0xFF):
            # print "Packet Rx error"
            pass
        pckt = getPcktMotorPwrM2(int(-right*scale))
        writeBytes(pckt,ser)
        sleep(0.05)
        temp = readBytesRaw(1,ser)
        if(temp != 0xFF):
            # print "Packet Rx error"
            pass
        #writeBytes([0x80,0,abs(left),
        #Get encoder1 state
        writeBytes([0x80,16],ser)
        sleep(0.05)
        temp = readBytesRaw(7,ser)
        enc1 = ctypes.c_int32((temp[0]<<24)|(temp[1]<<16)|(temp[2]<<8)|(temp[3])).value
        
        #Get encoder2 state
        writeBytes([0x80,17],ser)
        sleep(0.05)
        temp = readBytesRaw(7,ser)
        enc2 = ctypes.c_int32((temp[0]<<24)|(temp[1]<<16)|(temp[2]<<8)|(temp[3])).value
        print "Encoders: %d %d" % ((enc1),(enc2))
        
        # brightness = toByte(-stick.get_axis(3))
        # if (stick.get_button(2)):
        #    tilt = 90
        # if (stick.get_button(3)):
        #    panSpeed = 93
        # hat = stick.get_hat(0)[0]
        # if hat != basePrev:
           # basePrev = hat
           # panSpeed += -1 * stick.get_hat(0)[0]



        # if (stick.get_button(0)):
        #    tiltAxis = stick.get_axis(1)
        #    tilt = cap(tilt + tiltAxis*1.5, 60, 130)
        # print "                "
        # # print "Brightness:", brightness, " " * 10
        # print "PanSpeed  :", panSpeed, " " * 10
        # print "Tilt      :", "%.3f" % tilt, " " * 10

        # if count == 1:
        #    wrote = False
           # if (brightness != prevLight):
           #     writeBytes([0, 1, brightness])
           #     prevLight = brightness
           #     wrote = True
           # # writeBytes([0, 2, brightness])
           # # writeBytes([0, 3, brightness])
           # # writeBytes([0, 4, brightness])
           # if (panSpeed != prevPan):
           #     writeBytes([0, 9, panSpeed])
           #     prevPan = panSpeed
           #     wrote = True

           # if (int(tilt) != prevTilt):
           #     writeBytes([0, 8, int(tilt)])
           #     prevTilt = int(tilt)
           #     wrote = True
        #    if wrote:
        #        print "#######    \r\033[5A"
        #    else :
        #        print "           \r\033[5A"

        #    count = 0
        # else:
        #    print "           \r\033[5A"
        #    count += 1
        sleep(.05)




if __name__ == '__main__':
    main()