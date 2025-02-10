#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
import json
import serial
from time import time

global run, srl, response, detected


def move_callback(delay):
    global run, srl, detected
    flag = True

    delay = delay.data

    info = f'1,{delay}'
    srl.write((info+'\n').encode()) #append newline charecter

    run = False


if __name__ == '__main__':
    #INITIALIZE VARIABLES
    srl = serial.Serial("/dev/ttyUSB0", 9600, timeout=1)
    run = True
    detected = False

    # ROS NODE INITIALIZE
    rospy.init_node('move_node', anonymous=True)

    # CREATING THE PUBLISHERS
    status_pub = rospy.Publisher('status', String, queue_size=10)
    arduino_response = rospy.Publisher('response', String, queue_size=10)

    # CREATING THE SUBSCRIBERS
    rospy.Subscriber('delay_info', String, move_callback)


# Control loop of Model
    while not rospy.is_shutdown():
        response = srl.readline().decode('utf-8', errors='ignore').strip()
        arduino_response.publish(response)
        
        if response == "run":
            run = True
            info = []
        
        elif response == '':
            pass

        elif response != "run":
            run = False

        status_pub.publish(f"{run}")

        if not detected:
            info = '2,'
            srl.write((info+'\n').encode()) #append newline charecter
