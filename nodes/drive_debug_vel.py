

# -*- coding: utf-8 -*-

import serial
import pygame
from time import sleep
from random import random
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



def dec2ascii(velocity):
    if(velocity >= 0):
        assert(0 <= velocity and velocity <= 1200)
        return "%08X" % velocity
    else:
        assert(-1200 <= velocity and velocity < 0)
        return "%08X" % (0xFFFFFFFF - (abs(velocity) - 1))


def setMotorVel(ser,m1_vel,m2_vel):
    ser.write(chr(0x7E))#Start delimiter
    ser.write(chr(0x09))#Size
    ser.write(chr(0x30))

    ser.write(chr((m1_vel>>24)&0xFF))#M1[31:24]
    ser.write(chr((m1_vel>>16)&0xFF))#M1[23:16]
    ser.write(chr((m1_vel>>8)&0xFF))#M1[15:8]
    ser.write(chr(m1_vel&0xFF))#M1[7:0]
    ser.write(chr((m2_vel>>24)&0xFF))#M2[31:24]
    ser.write(chr((m2_vel>>16)&0xFF))#M2[23:16]
    ser.write(chr((m2_vel>>8)&0xFF))#M2[15:8]
    ser.write(chr(m2_vel&0xFF))#M2[7:0]


def getEncoderVel():
    if(readBytesRaw(1,ser) == '0x55'):
        if(readBytesRaw(1,ser) == '0x08'):
            enc1 = readBytesRaw(1,ser)<<24 + readBytesRaw(1,ser)<<16 + readBytesRaw(1,ser)<<8 + readBytesRaw(1,ser);
            enc2 = readBytesRaw(1,ser)<<24 + readBytesRaw(1,ser)<<16 + readBytesRaw(1,ser)<<8 + readBytesRaw(1,ser);
            if(readBytesRaw(1,ser) == '0x33'):
                return (enc1,enc2)
    return None

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
    print "[Joystick] %d hats" % stick.get_numhats()
    ser = serial.Serial("/dev/ttyUSB0",115200,timeout=0)
    ser.flushInput()
    ser.flushOutput()
    while True:
        #Get joystick state
        pygame.event.pump()
        axis_x = toByte(stick.get_axis(0))-128
        axis_y = -(toByte(stick.get_axis(1))-128)
        axis_z = -(toByte(stick.get_axis(2))-128)
        #Mixing
        left = cap(axis_x+axis_y,-127,127)
        right = cap(-axis_x+axis_y,-127,127)
        scale = float(128+cap(axis_z,-128,127))
        #print str(axis_x)+" "+str(axis_y)+" "+str(axis_z)
        #print str(left)+" "+str(right)
        m1_vel = cap(int(left*scale),-1200,1200)
        m2_vel = cap(int(right*scale),-1200,1200)

        setMotorVel(ser,m1_vel,m2_vel)
        sleep(.05)
        """
        ser.write(chr(13))  #CR
        #ser.write(chr(10))  #LF to get new terminal prompt
        ser.write('r')
        ser.write('c')
        ser.write(' ')
        ser.write('v')
        ser.write('1')
        ser.write(' ')
        ser.write(dec2ascii(cap(int(left*scale),-1200,1200)))
        ser.write(chr(13))  #CR
        sleep(0.05)
        readBytesRaw(21,ser)
        temp = readBytesRaw(3,ser)
        readBytesRaw(20,ser)
        #print ser.read(50)
        if(temp != [97,99,107]):
            print "Packet Rx error1"
        ser.write(chr(13))  #CR
        ser.write('r')
        ser.write('c')
        ser.write(' ')
        ser.write('v')
        ser.write('2')
        ser.write(' ')
        ser.write(dec2ascii(cap(int(right*scale),-1200,1200)))
        ser.write(chr(13))  #CR
        sleep(0.05)
        readBytesRaw(21,ser)
        temp = readBytesRaw(3,ser)
        readBytesRaw(20,ser)
        if(temp != [97,99,107]):
            print "Packet Rx error2"
        #print ser.read(50)
        #Get encoder values
        ser.write('encoders')
        ser.write(chr(13))  #CR
        sleep(0.05)
        readBytesRaw(12,ser)
        print ser.read(19)
        readBytesRaw(20,ser)
"""


if __name__ == '__main__':
    main()
