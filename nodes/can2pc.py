#!/usr/bin/env python
import serial
import rospy
from std_msgs.msg import String

def get_can():
        
    #Configure serial port
    #115200 baud, 8 bits, 1 stop, no parity
    ser = serial.Serial('/dev/ttyUSB0',baudrate=115200,timeout=0.1)
    ser.close() #Close the port if already open
    ser.open()
    i = 0
    j = 0
    pub = rospy.Publisher('chatter', String, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(30) # 10hz
    rx_msg = "HELLO WORLD"
    while not rospy.is_shutdown():
        #Check if there is data to tx
        ser.write(chr(13))#CR
        ser.write('can tx 0x0')
        ser.write(chr(ord('0')+j))
        ser.write(chr(13))#CR
        ser.write(chr(13))#CR
        ser.write('can rx')
        ser.write(chr(13))#CR
        j = j + 1
        if(j == 10):
            j = 0

        buf = ser.read(35)
        idx = buf.find('can rx')
        if(idx >= 0):
            try:
                rx_byte = buf[idx+8:idx+10]
                
                rx_num = int(rx_byte,16)
                rx_msg = str(rx_num)
                print(rx_msg)
                # print "received"+str(rx_num)
            except:
                if(rx_byte == 'na'):
                    pass
                    # print "No Data"
                rx_msg = str("")
        else:
            rx_msg = ""
        pub.publish(rx_msg)
        rate.sleep()


if __name__ == '__main__':
    try:
        get_can()
    except rospy.ROSInterruptException:
        pass