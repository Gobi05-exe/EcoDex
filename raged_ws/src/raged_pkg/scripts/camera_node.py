#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

def main():
    # Initialize the ROS node
    rospy.init_node('camera_node', anonymous=True)
    image_pub = rospy.Publisher('/camera/image_raw', Image, queue_size=10)
    bridge = CvBridge()

    # Open the camera (usually /dev/video0 for USB cameras)
    cap = cv2.VideoCapture(2)

    if not cap.isOpened():
        rospy.loginfo("Camera not found.")
        return

    while not rospy.is_shutdown():
        ret, frame = cap.read()
        if ret:
            # Convert OpenCV image to ROS Image message
            ros_image = bridge.cv2_to_imgmsg(frame, "bgr8")
            image_pub.publish(ros_image)

    # Release the camera when done
    cap.release()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
