#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import String
import json
from ultralytics import YOLO  
import cv2
from time import time


global bridge, ahead, results, thres_box_frame_ratio, frame_height, frame_width, run, detected


def get_box_size(box):
    x1, y1, x2, y2 = box.xyxy[0]
    box_width = abs(x2 - x1)
    box_height = abs(y2 - y1)
    box_size = (box_width**2 + box_height**2)**0.5
    return box_size


def get_closest_object_info():
    global results, ahead
    box_info = {}

    if results[0].boxes is not None:
        boxes = results[0].boxes
        on_off.publish("Detected")
        end_t = time()+2

        while time()<end_t:
            results[0].boxes = None
        
        closest_object = boxes[0]
        for box in boxes:
            if get_box_size(box) > get_box_size(closest_object):
                closest_object = box

        cls_id = int(closest_object.cls[0])
        class_name = results[0].names[cls_id]
        box_info['Class'] = class_name
        x1, _, x2, _ = closest_object.xyxy[0]

        box_info['Centre'] = (frame_width*0.5, frame_height*0.5)
        box_info['Object_pos'] = [((float(x1)+float(x2))/2), (frame_height/2)]
        box_info['Ahead'] = ahead
        box_info['Detected'] = True

        if class_name in ["METAL", "GLASS", "PLASTIC"]:
            box_info['isBiodegradable'] = False
        else:
            box_info['isBiodegradable'] = True

    else:
        on_off.publish("Not Detected")
        detected = False
        box_info['Ahead'] = ahead
        box_info['Detected'] = False

    box_info = json.dumps(box_info)
    move_pub.publish(box_info)

    return None


def remove_large(frame_size):
    global results, thres_box_frame_ratio

    keep_boxes = []

    for i, box in enumerate(results[0].boxes):
        box_size = get_box_size(box)
        box_frame_ratio = box_size / frame_size

        if box_frame_ratio <= thres_box_frame_ratio:
            keep_boxes.append(i)

    if keep_boxes:
        results[0].boxes = results[0].boxes[keep_boxes]

    else:
        results[0].boxes = None
    


def status_callback(cam_status):
    global ahead, run
    rospy.loginfo("Runiing...")
    rospy.loginfo(f"{(cam_status.data)}, Detected: {detected}")
    cam_status = json.loads(cam_status.data)
    ahead = cam_status['Ahead']
    run = cam_status['Run']

def image_callback(ros_image, model):
    global bridge, results
    global frame_height, frame_width
    global ahead, run
    
    if run:
        # Convert the ROS image message to an OpenCV image using cv_bridge
        frame = bridge.imgmsg_to_cv2(ros_image, "bgr8")

        # Get frame size for object size comparisons
        frame_height, frame_width, _ = frame.shape
        frame_size = (frame_width**2 + frame_height**2)**0.5

        results = model(frame, imgsz=512, save=False, plots=False, device='cpu', verbose=False)

        '''if ahead:
            remove_large(frame_size)

        elif not ahead:
            get_closest_object_info()'''
        
        remove_large(frame_size)
        get_closest_object_info()

        '''annotated_frame = results[0].plot()
        cv2.imshow("Garbage Detection", annotated_frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            cv2.destroyAllWindows()'''
        #on_off.publish("ON")

        
    else:
        #cv2.destroyAllWindows()
        #on_off.publish("OFF")
        pass


if __name__ == '__main__':
    #INITIALIZE VARIABLES
    ahead = True # only intial value is set in this program
    run = True # only intial value is set in this program
    detected = False # only intial value is set in this program

    bridge = CvBridge()
    model_path = "runs/detect/final_train/weights/best_openvino_model"
    model = YOLO(model_path)
    thres_box_frame_ratio = 0.8

    # ROS NODE INITIALIZE
    rospy.init_node('ml_node', anonymous=True)

    # CREATING THE PUBLISHERS
    move_pub = rospy.Publisher('detected_info', String, queue_size=10)
    on_off = rospy.Publisher('onoff', String, queue_size=10)

    # CREATING THE SUBSCRIBERS
    rospy.Subscriber('/camera/image_raw', Image, image_callback, callback_args=(model))
    rospy.Subscriber('status', String, status_callback)

    #ROS Spin
    rospy.spin()

#ROS TOPICS
'''
    1. /camera/image_raw
    2. /status               (for the position of the camera -> ahead, down or detected)
    3. /detected_info        (object detected in the frame along with the camera pos info)
    4. /onoff                (tells whether the model is running or not)
    5. /response             (for the serial resonse from the arduino or serial monitoring)
'''