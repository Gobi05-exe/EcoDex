#!/usr/bin/env python3
import rospy
from std_msgs.msg import String
import json
from pymongo import MongoClient
from os import system


def msgCallBack(info):
    rospy.loginfo("%s", info.data)
    record = json.loads(info.data)
    collection = db[record['Username']]
    del record['Username']
    collection.insert_one(record)
    return

if __name__ == '__main__':
    # Replace with your MongoDB URI
    client = MongoClient('mongodb+srv://arjundevraj05:arjun123@cluster0.ev0ma.mongodb.net/?retryWrites=true&w=majority&appName=Cluster0')                      
    db = client["test"]

    rospy.init_node("web_node", anonymous=True)

    rospy.Subscriber("delay_info", String, queue_size=10)

    rospy.spin()
    
'''
    ROS TOPICS
    1. detected_info        (object detected in the frame along with the camera pos info)
'''

# mongodb+srv://arjundevraj05:arjun123@cluster0.ev0ma.mongodb.net/?retryWrites=true&w=majority&appName=Cluster0