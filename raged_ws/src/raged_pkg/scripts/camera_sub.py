#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2

global run
run = True

def image_callback(img_msg):
    global run

    if run:
        cv_image = bridge.imgmsg_to_cv2(img_msg)
        cv_image = cv2.flip(cv_image, 1)
        cv2.imshow("Image Window", cv_image)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            run = False
    else:
        cv2.destroyAllWindows()
        rospy.loginfo("Not Running...")


if __name__ == '__main__':
    bridge = CvBridge()
    rospy.init_node('camera_sub', anonymous=True)
    sub_image = rospy.Subscriber("/camera/image_raw", Image, image_callback)
    rospy.spin()