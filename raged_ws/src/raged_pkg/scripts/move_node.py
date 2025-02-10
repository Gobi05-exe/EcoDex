#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
import json
import serial
from time import time

global ahead, run, srl, response, status


def move_callback(object_info):
    global run, srl, ahead, status
    status = {}

    object_info = json.loads(object_info.data)


    if ahead and object_info['Detected']:
        #camera down and turn to and go to object
        #dist = object_info['Object_pos'] - object_info['Centre']
        dist = 5
        info = f'1,{dist}'
        info = ','.join(map(str,info))
        srl.write((info+'\n').encode()) #append newline charecter

        ahead = False
        run = False

    elif not ahead and object_info['Detected']:
        #continue moving ahead slowly until object detected
        #write run in the starting of the movement block
        info = '4,'
        srl.write((info+'\n').encode()) #append newline charecter

    elif ahead and not object_info['Detected']:
            #continue moving ahead
            info = '2,'
            srl.write((info+'\n').encode()) #append newline charecter

    elif not ahead and not object_info['Detected']:
        #collect the waste and move camera up
        ahead = True
        info = '3,'
        srl.write((info+'\n').encode()) #append newline charecter


if __name__ == '__main__':
    #INITIALIZE VARIABLES
    srl = serial.Serial("/dev/ttyACM0", 9600, timeout=1)
    run = True
    ahead = True
    status = {}

    # ROS NODE INITIALIZE
    rospy.init_node('move_node', anonymous=True)

    # CREATING THE PUBLISHERS
    status_pub = rospy.Publisher('status', String, queue_size=10)
    arduino_response = rospy.Publisher('response', String, queue_size=10)

    # CREATING THE SUBSCRIBERS
    rospy.Subscriber('detected_info', String, move_callback)

    #INITIAL PUBLISH
    status['Ahead'] = ahead
    status['Run'] = run


# Control loop of Model
    while not rospy.is_shutdown():
        status['Ahead'] = ahead
        status['Run'] = run
        status_pub.publish(json.dumps(status))
        response = srl.readline().decode('utf-8', errors='ignore').strip()
        arduino_response.publish(response)
        
        if response == "run":
            run = True
            object_info = {}
            info = []
        
        elif response == '':
            pass

        elif response != "run":
            run = False

