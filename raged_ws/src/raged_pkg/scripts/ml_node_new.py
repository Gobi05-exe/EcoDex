#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
from ultralytics import YOLO
import time
import geocoder
from datetime import datetime
import json

# Initialize the CvBridge
bridge = CvBridge()

# Load the YOLO model
model = YOLO("yolov8n.pt")  # Use a YOLO model trained on the COCO dataset

# Initialize the global variables
run = False
last_detection_time = 0  # To keep track of the last detection time

# Status callback from Arduino (control of model)
def status_callback(cam_status):
    global run
    cam_status = eval(cam_status.data)
    run = cam_status

# Function to process the image and filter for "bottle"
def image_callback(ros_image):
    global run, last_detection_time

    if run:
        try:
            # Ensure detection happens only once every second
            current_time = time.time()
            if current_time - last_detection_time < 1.0:
                return  # Skip processing if less than 1 second has passed

            # Update the last detection time
            last_detection_time = current_time

            # Convert the ROS image message to an OpenCV image
            frame = bridge.imgmsg_to_cv2(ros_image, "bgr8")

            # Perform YOLO inference on the frame
            results = model(frame, imgsz=640, device="cpu")  # Use GPU if available

            # Extract results for "bottle" (class ID: 39)
            bottle_detections = [
                box for box in results[0].boxes if int(box.cls[0]) == 39       #temp change
            ]

            # Find the largest bounding box based on area
            if bottle_detections:
                web_info = {}
                location = geocoder.ip("me") #current location
                coordinates = [location.lat, location.lng]
                today = datetime.today()
                date = str(today.strftime("%d/%m/%y"))
                day = str(today.strftime("%A"))

                largest_box = max(bottle_detections, key=lambda box: (box.xyxy[0][2] - box.xyxy[0][0]) * (box.xyxy[0][3] - box.xyxy[0][1]))

                # Extract bounding box coordinates
                x1, y1, x2, y2 = map(int, largest_box.xyxy[0])  
                confidence = largest_box.conf[0]
                
                class_id = 39 #int(largest_box.cls[0])  # Extracts the first element and converts to int
                class_name = results[0].names[class_id]      #temp change
                
                isBiodegradable = False
                if class_name in ["PLASTIC", "METAL", "GLASS"]:
                    isBiodegradable = False
                else:
                    isBiodegradable = True

                # Adding the values to the dict
                web_info['Class'] = str(class_name).upper()
                web_info['isBiodegradable'] = isBiodegradable
                web_info['Latitude'] = float(coordinates[0])
                web_info['Longitude'] = float(coordinates[1])
                web_info['Date'] = str(date)
                web_info['Day'] = str(day)
                web_info = json.dumps(web_info)


                # Calculate pixel width
                pixel_width = x2 - x1

                # Calculate distance
                if pixel_width > 0:
                    distance = (6.5 * 527.5) / pixel_width
                    distance_text = f"{distance:.2f} cm"
                    delay = distance * 0.07
                    move_pub.publish(f"{delay}")
                    web_pub.publish(web_info)
                else:
                    distance_text = "N/A"

                # Draw bounding box and label with distance
                cv2.rectangle(frame, (x1, y1), (x2, y2), (255, 0, 0), 2)
                cv2.putText(
                    frame,  
                    f"{class_name} {confidence:.2f}, {distance_text}",
                    (x1, y1 - 10),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.5,
                    (255, 0, 0),
                    2,
                )

                # Publish detection message
                on_off.publish("Detected")

                # ROS log
                rospy.loginfo(
                    f"Largest Bottle Detected: x1={x1}, y1={y1}, x2={x2}, y2={y2}, confidence={confidence:.2f}, width={pixel_width}px, distance={distance_text}"
                )

            # Display the frame
            cv2.imshow("Detection", frame)

            # Exit on 'q' key press
            if cv2.waitKey(1) & 0xFF == ord('q'):
                rospy.signal_shutdown("User requested shutdown")
                cv2.destroyAllWindows()
        except Exception as e:
            rospy.logerr(f"Error processing image: {e}")
    else:
        cv2.destroyAllWindows()
        on_off.publish("OFF")
        return


if _name_ == "_main_":
    
    # ROS NODE INITIALIZE
    rospy.init_node('ml_node', anonymous=True)

    # CREATING THE PUBLISHERS
    move_pub = rospy.Publisher('delay_info', String, queue_size=10)
    on_off = rospy.Publisher('onoff', String, queue_size=10)
    web_pub = rospy.Publisher('detected_info', String, queue_size=10)

    # CREATING THE SUBSCRIBERS
    rospy.Subscriber('/camera/image_raw', Image, image_callback)
    rospy.Subscriber('status', String, status_callback)

    # ROS Spin
    rospy.spin()
