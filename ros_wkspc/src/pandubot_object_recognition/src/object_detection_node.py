#!/usr/bin/env python 

import rospy
from sift import SIFTMatching
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import cv2
import object_action_server

# def callback_image(image_msg):
#   print 'image cb'
#   bridge = CvBridge()
#   cv2_image = bridge.imgmsg_to_cv2(image_msg,"bgr8")
#   cv2_image = np.array(cv2_image,dtype=np.uint8)
#   #cv2.waitKey(1000/30)
#   ## IsMatch takes in a grayscale image
#   cv_image_gray = cv2.cvtColor(cv2_image,cv2.COLOR_BGR2GRAY)
#   cv_image_gray = np.array(cv_image_gray,dtype=np.uint8)
#   label = SIFTMatching.IsMatch(cv_image_gray)
#   #cv2.imshow('debug',cv_image_gray)
#   print label




def run_object_detection():
  print 'hello'
  rospy.init_node("pandubot_object_detection",anonymous=False)
  object_action_server.ObjectAction(rospy.get_name())
  #rospy.Subscriber("/usb_cam/image_raw",Image,callback_image)
  rospy.spin()
  

if __name__ == '__main__':
  run_object_detection()
  print "hello out main" 
  
  
  
    
