#!/usr/bin/env python
import roslib;
roslib.load_manifest('can2pc')
import rospy
from std_msgs.msg import String
from std_msgs.msg import Float32
from geometry_msgs.msg import Pose2D
from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import Point
import tf
from tf import TransformerROS
import math
import serial
from time import sleep
from random import random
import ctypes

class Robot_Com(object):
    """docstring for Robot_Com"""
    def __init__(self):
        rospy.init_node('roboclaw_serial', anonymous=True)
        rospy.Subscriber("/yaw", Float32, self.yaw_callback)
        rospy.Subscriber("/eliminate",String,self.armEliminator)
        self.br = tf.TransformBroadcaster()

        self.ser = serial.Serial("/dev/ttyUSB0",115200,timeout=0.1)
        self.ser.flushInput()
        self.ser.flushOutput()
        
        self.init_encoders();
        
        self.ser.flushInput()
        self.ser.flushOutput()
        
        self.init_Eliminator();
        self.theta = 0;
        self.x = 7;
        self.y = 7;
        self.z = 0;
        self.eliminate = False
        # self.theta = 0;
        self.t_init = rospy.Time.now().to_sec();
        self.t_prev = rospy.Time.now().to_sec();
        self.t = rospy.Time.now().to_sec();
        self.left = 0
        self.right = 0

        self.wheel_diameter = .254 # wheel diameter in m
        self.wheel_base = 0.56261; # m
        self.tics_per_rev = 2000

        self.camera_height = .4;
        
        self.t1 = rospy.Time.now()
        self.t0 = rospy.Time.now()
        self.pos1 = 0
        self.pos2 = 0
        self.max_vel = .3;
        self.max_w = 0.6;
      
        rospy.Subscriber("/cmd", Pose2D, self.cmd_callback)

        rate = rospy.Rate(20)
        self.set_pid(4000,0,4000)
        while not rospy.is_shutdown():
            # print("VELOCITIES:", self.left,self.right)
            print(self.eliminate)
            if(self.eliminate):
                self.setMotorVel(0,0);
                self.activateEliminator()
                self.eliminate = False
                    
            self.setMotorVel(self.left,self.right)
            self.getEncoder()
            self.estimate_position()
            self.ser.flushInput()
            rate.sleep()    
        
        # STOP WHEN CLOSED
        self.setMotorVel(0,0)
    
    def yaw_callback(self,data):
        self.theta = data.data


    def init_encoders(self):
        self.encoder1 = 0;
        self.encoder2 = 0;
        encoder_data = False
        while(not encoder_data):
            encoder_data = self.getEncoder()
        self.encoder1_offset = self.encoder1
        self.encoder2_offset = self.encoder2

    def estimate_position(self):
        self.t0 = self.t1
        self.t1 = rospy.Time.now()

        self.prev_pos1 = self.pos1
        self.prev_pos2 = self.pos2
        self.pos1 = (self.wheel_diameter*math.pi)*(self.encoder1-self.encoder1_offset)/(self.tics_per_rev)
        self.pos2 = (self.wheel_diameter*math.pi)*(self.encoder2-self.encoder2_offset)/(self.tics_per_rev)
        dp1 = self.pos1 - self.prev_pos1
        dp2 = self.pos2 - self.prev_pos2
        dt = (self.t1-self.t0).to_sec()
        vl = dp1/dt
        vr = dp2/dt

        V = (vl + vr)/2.0
        w = (vr - vl)/self.wheel_base
        dp = (dp1 + dp2)/2.0
        if(math.fabs(V)>10):
            print("BAD VEL")
            # exit()
            self.setMotorVel(0,0)
            self.init_encoders()
            return
        self.x = self.x + math.cos(self.theta)*dp
        self.y = self.y + math.sin(self.theta)*dp
        # self.theta = (self.theta + w*dt) % (2*math.pi)

        print(vl,vr)
        print(self.x,self.y,math.degrees(self.theta))

        self.br.sendTransform((self.x, self.y, 0),
                 tf.transformations.quaternion_from_euler(0, 0, self.theta),
                 rospy.Time.now(),
                 "/robot",
                 "/world")                       
        
        self.br.sendTransform((0, 0, self.camera_height),
                 tf.transformations.quaternion_from_euler(0, 0, -3.14),
                 rospy.Time.now(),
                 "/camera",
                 "/robot")

        self.br.sendTransform((-.1, 0, 0),
                 tf.transformations.quaternion_from_euler(0, 0, 0),
                 rospy.Time.now(),
                 "/eliminator",
                 "/robot") 


    def cmd_callback(self, data):
        print("IN CALLBACK")
        print(data)
        v = data.x;
        w = data.theta;
        if(math.fabs(v)>self.max_vel):
            v = math.copysign(self.max_vel,v)
        if(math.fabs(w)>self.max_w):
            w = math.copysign(self.max_w,w)
        self.left = v-w*self.wheel_base/2;
        self.right= v+w*self.wheel_base/2;
        print("Left: %f \t Right: %f" % (self.left, self.right) )

    def set_pid(self,kp,ki,kd):
        print(kp)
        print(dec2ascii(kp*(16**4)))
        self.ser.write('+++')
        self.ser.write('\r')
        self.ser.write('rc pid1 ')
        self.ser.write((dec2ascii(kd*(16**4))))
        self.ser.write(' ')
        self.ser.write((dec2ascii(kp*(16**4))))
        self.ser.write(' ')
        self.ser.write((dec2ascii(ki*(16**4))))
        self.ser.write(' ')
        self.ser.write('00001200')
        self.ser.write('\r')
        
        self.ser.write('rc pid2 ')
        self.ser.write((dec2ascii(kd*(16**4))))
        self.ser.write(' ')
        self.ser.write((dec2ascii(kp*(16**4))))
        self.ser.write(' ')
        self.ser.write((dec2ascii(ki*(16**4))))
        self.ser.write(' ')
        self.ser.write('00001200')
        self.ser.write('\r')

        self.ser.write('exit\r')

                        



    def vel_to_qps(self,left,right):
        left_qps = int(self.tics_per_rev*left/(self.wheel_diameter*math.pi))
        right_qps = int(self.tics_per_rev*right/(self.wheel_diameter*math.pi))
        return(left_qps,right_qps)


    def setMotorVel(self,m1_vel,m2_vel):
        (m1_vel,m2_vel) = self.vel_to_qps(m1_vel,m2_vel)
        # print(m1_vel,m2_vel)
        self.ser.write(chr(0x7E))#Start delimiter
        self.ser.write(chr(0x09))#Size
        self.ser.write(chr(0x30))

        self.ser.write(chr((m1_vel>>24)&0xFF))#M1[31:24]
        self.ser.write(chr((m1_vel>>16)&0xFF))#M1[23:16]
        self.ser.write(chr((m1_vel>>8)&0xFF))#M1[15:8]
        self.ser.write(chr(m1_vel&0xFF))#M1[7:0]
        self.ser.write(chr((m2_vel>>24)&0xFF))#M2[31:24]
        self.ser.write(chr((m2_vel>>16)&0xFF))#M2[23:16]
        self.ser.write(chr((m2_vel>>8)&0xFF))#M2[15:8]
        self.ser.write(chr(m2_vel&0xFF))#M2[7:0]


    def getEncoder(self):
        s = readBytesRaw(1,self.ser);
        # print(s[0])
        # print(0x55)
        # print(s)
        if(s[0] == 0x55):
            s = readBytesRaw(1,self.ser);
            if(s[0] == 0x0A):
                e1 = readBytesRaw(4,self.ser);
                e2 = readBytesRaw(4,self.ser);
                sensor = readBytesRaw(1,self.ser);
                batteryVoltage = readBytesRaw(1,self.ser);
                end = readBytesRaw(1,self.ser);
                if(end[0] == 0x33):
                    enc1 = e1[0]*2**24 + e1[1]*2**16 + e1[2]*2**8 + e1[3];
                    enc2 = e2[0]*2**24 + e2[1]*2**16 + e2[2]*2**8 + e2[3];

                    self.encoder1 = int(ctypes.c_int32(enc1).value)
                    self.encoder2 = int(ctypes.c_int32(enc2).value)
                    
                    self.sensor_signal = sensor[0] & 0x30
                    self.batteryVoltage = batteryVoltage

                    # print(encoder1,encoder2)
                    # if(math.fabs(self.encoder1-encoder1)>1000 or math.fabs(self.encoder2-encoder2)>1000):
                        # print("ENCODER FAIL")
                        # return False
                    return True
        return False

    def init_Eliminator(self):
        self.ser.write('+++')
        self.ser.write('\r')
        print("HOMING")
        self.ser.write('step home\r')
        d = rospy.Duration(15);
        rospy.sleep(d)
        self.ser.write('exit\r')

    def armEliminator(self,data):
        print("ARMING ELIMINATOR")
        self.eliminate = True
        
    def activateEliminator(self):
        self.ser.write('+++')
        self.ser.write('\r')
        print("ELIMINATING")
        d_3 = rospy.Duration(3);
        d_10= rospy.Duration(10);
        self.ser.write('drill dir 2\r')
        
        self.ser.write('step pos 0729\r')
        
        rospy.sleep(d_3)
        self.ser.write('drill dir 2\r')
        rospy.sleep(d_3)
        rospy.sleep(d_3)
        
        self.ser.write('step pos 0100\r')
        self.ser.write('drill off\r')
        rospy.sleep(d_3)

        self.ser.write('drill dir 1\r')
        rospy.sleep(d_3)
        self.ser.write('drill off\r')
        # rospy.sleep(d_5)
        self.ser.write('exit\r')

                        


def dec2ascii(velocity):
    if(velocity >= 0):
        return "%08X" % velocity
    else:
        return "%08X" % (0xFFFFFFFF - (abs(velocity) - 1))


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
    
    try:
        raw_string = ser.read(num)
    except:
        print('FAIL')
        return [None]
    raw_bytes = len(raw_string)*[0]
    for i in range(0,len(raw_string)):
        raw_bytes[i] = ord(raw_string[i])
    if(len(raw_bytes) == 0):
        raw_bytes = [None]
    return raw_bytes

# def calcCRC(byteArray):
#     crc = 0
#     for i in range(0,len(byteArray)):
#         crc = crc ^ (byteArray[i]<<8)
#         for j in range(0,8):
#             if(crc & 0x8000):
#                 crc = ((crc << 1) & 0xFFFF) ^ 0x1021
#             else:
#                 crc = (crc << 1) & 0xFFFF
#     return crc

# def getPcktMotorPwrM1(velocity):
#     if(velocity >= 0):
#         assert(0 <= velocity and velocity <= 127)
#         pckt = [0x80,0,velocity]
#         crc = calcCRC(pckt)
#         return pckt+[(crc>>8)&0xFF,crc&0xFF]
#     else:
#         assert(-127 <= velocity and velocity <= 0)
#         pckt = [0x80,1,abs(velocity)]
#         crc = calcCRC(pckt)
#         return pckt+[(crc>>8)&0xFF,crc&0xFF]

# def getPcktMotorPwrM2(velocity):
#     if(velocity >= 0):
#         assert(0 <= velocity and velocity <= 127)
#         pckt = [0x80,4,velocity]
#         crc = calcCRC(pckt)
#         return pckt+[(crc>>8)&0xFF,crc&0xFF]
#     else:
#         assert(-127 <= velocity and velocity <= 0)
#         pckt = [0x80,5,abs(velocity)]
#         crc = calcCRC(pckt)
#         return pckt+[(crc>>8)&0xFF,crc&0xFF]

        


if __name__ == '__main__':
    com1 = Robot_Com();