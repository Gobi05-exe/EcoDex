#!/usr/bin/env python3
import rospy
from std_msgs.msg import String
import json
from pymongo import MongoClient

def msgCallBack(info):
#    rospy.loginfo("%s", info.data)
    record = {}
    record = json.loads(info.data)

    rospy.loginfo(f"{record}")
    collection = db['Jacob_waste_records']
    collection.insert_one(record)
    return

if _name_ == '_main_':
    # Replace with your MongoDB URI
    client = MongoClient(r'mongodb+srv://arjundevraj05:arjun@cluster0.ev0ma.mongodb.net/?retryWrites=true&w=majority')                      
    db = client["test"]

    rospy.init_node("web_node", anonymous=True)

    rospy.Subscriber("detected_info", String, msgCallBack)
    
    rospy.spin()

'''
    ROS TOPICS
    1. detected_info        (object detected in the frame along with the camera pos info)
'''

# mongodb+srv://arjundevraj05:arjun123@cluster0.ev0ma.mongodb.net/?retryWrites=true&w=majority&appName=Cluster0
# MONGODB_URI=mongodb+srv://arjundevraj05:arjun@cluster0.ev0ma.mongodb.net/
