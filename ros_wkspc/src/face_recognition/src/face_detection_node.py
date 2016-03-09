#!/usr/bin/env python

import faceDetect as fd
import rospy
import cv2
import numpy as np
from cv_bridge import CvBridge,CvBridgeError
from sensor_msgs.msg import Image


classifier_xml_dir = '/home/bhavya/pandubot_wkspc/ros_wkspc/src/face_recognition/src/haarcascades/haarcascade_frontalface_alt.xml'

def callback(image_msg):
  bridge = CvBridge();
  cv2_image = bridge.imgmsg_to_cv2(image_msg,"bgr8")
  cv2_image = np.array(cv2_image,dtype=np.uint8) 
  (bbox,detected_flag) = fd.faceDetect(cv2_image,classifier_xml_dir);
  cv2.imshow('image_subscriber',cv2_image)
  cv2.waitKey(1000/30);
  # rospy.loginfo(str(detected_flag))
  print detected_flag


def listener():
  rospy.init_node("image_subscriber",anonymous=True)
  rospy.Subscriber("/webcam/image_raw",Image,callback);
  rospy.spin()

if __name__=='__main__':
  listener()