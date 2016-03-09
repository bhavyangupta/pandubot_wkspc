#!/usr/bin/env python 

import rospy
from sensor_msgs.msg import Image

#def callback_image(image):
#  print 'image callback'


def run_object_detection():
  print 'hello'
  rospy.init_node("pandubot_object_detection",anonymous=True)
#  rospy.Subscriber("/usb_cam/image_raw",Image,callback_image)
  rospy.spin()
  

if __name__ == '__main__':
  print "hello siddharth"
  run_object_detection()
  print "hello out main" 
  
  
  
    
