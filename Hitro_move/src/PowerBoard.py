#!/usr/bin/env python3
#coding: utf-8
import rospy
from std_msgs.msg import Int16MultiArray
import serial
import time

def init():
    global ser
    try:
        ser = serial.Serial('/dev/ttyUSB0',115200,timeout=0.1)
        #venderid: 0x0403 productid: 0x6001
    except:
        print("Connection Failed")
        quit()

    print("Connection Succeeded")

init()


def callback(msg):
    global ser
    for k in range(2):
        for i in range(len(msg.data)):
            if(msg.data[i]==1):
                string = "ON "+ str(i) + "-"
                ser.write(string.encode())
            else:
                string = "OFF "+ str(i) + "-"
                ser.write(string.encode())
            time.sleep(0.05)
    print("complete")
    time.sleep(0.1)


rospy.init_node("PowerBoard_controller")
rospy.Subscriber("Power_output_array",Int16MultiArray,callback)
rospy.spin()



