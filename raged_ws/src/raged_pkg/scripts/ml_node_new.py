#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
from ultralytics import YOLO

# Initialize the CvBridge
bridge = CvBridge()

# Load the YOLO model
model = YOLO("yolov8n.pt")  # Use a YOLO model trained on the COCO dataset

# status from arduino (control of model)
def status_callback(cam_status):
    global run
    cam_status = bool(cam_status.data)
    run = cam_status

# Function to process the image and filter for "bottle"
def image_callback(ros_image):
    global run

    if run:
        try:
            # Convert the ROS image message to an OpenCV image
            frame = bridge.imgmsg_to_cv2(ros_image, "bgr8")

            # Perform YOLO inference on the frame
            results = model(frame, imgsz=640, device="cpu")  # Use GPU if available

            # Extract results for "bottle" (class ID: 39)
            bottle_detections = [
                box for box in results[0].boxes if int(box.cls[0]) == 39
            ]

            # Find the largest bounding box based on area
            if bottle_detections:
                largest_box = max(bottle_detections, key=lambda box: (box.xyxy[0][2] - box.xyxy[0][0]) * (box.xyxy[0][3] - box.xyxy[0][1]))

                # Extract bounding box coordinates
                x1, y1, x2, y2 = map(int, largest_box.xyxy[0])  
                confidence = largest_box.conf[0]  
                class_name = results[0].names[39]  

                # Calculate pixel width
                pixel_width = x2 - x1

                # Calculate distance
                if pixel_width > 0:
                    distance = (6.5 * 527.5) / pixel_width
                    distance_text = f"{distance:.2f} cm"
                    delay = distance * 0.8
                    move_pub.publish(f"{delay}")
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



if __name__ == "__main__":
    #INITIALIZE VARIABLES
    run = True # only intial value is set in this program

    # ROS NODE INITIALIZE
    rospy.init_node('ml_node', anonymous=True)

    # CREATING THE PUBLISHERS
    move_pub = rospy.Publisher('delay_info', String, queue_size=10)
    on_off = rospy.Publisher('onoff', String, queue_size=10)

    # CREATING THE SUBSCRIBERS
    rospy.Subscriber('/camera/image_raw', Image, image_callback)
    rospy.Subscriber('status', String, status_callback)

    #ROS Spin
    rospy.spin()