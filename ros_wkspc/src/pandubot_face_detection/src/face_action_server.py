import roslib
import rospy
import actionlib
import pandubot_face_detection.msg
import cv2
import faceDetect
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

classifier_xml_dir = '/home/bhavya/pandubot_wkspc/ros_wkspc/src/face_recognition/src/haarcascades/haarcascade_frontalface_alt.xml'

def image_msg_to_grayscale(image_msg):
  print 'convt to gray' 
  bridge = CvBridge()
  cv_image = bridge.imgmsg_to_cv2(image_msg,"bgr8")
  cv_image = np.array(cv_image,dtype=np.uint8)
  cv_image_gray = cv2.cvtColor(cv_image,cv2.COLOR_BGR2GRAY)
  cv_image_gray = np.array(cv_image_gray,dtype=np.uint8)
  return cv_image_gray  


class FaceAction(object):
  feedback_ = pandubot_face_detection.msg.face_detectFeedback()
  result_ = pandubot_face_detection.msg.face_detectResult()

  def __init__(self,name):
    self.action_name_ = name
    self.action_server_ = actionlib.SimpleActionServer(self.action_name_, 
                                                       pandubot_face_detection.msg.face_detectAction,
                                                       execute_cb = self.execute_cb,
                                                       auto_start=False)
    self.action_server_.start()
    print 'Started Face Server'

  def execute_cb(self,goal):
    print 'goal rx'
    self.feedback_.busy_code = 0
    face_found = 0 
    while not face_found: 
      self.action_server_.publish_feedback(self.feedback_)
      print '[FD] wait for image'
      image_msg = rospy.wait_for_message("/usb_cam/image_raw",Image) 
      print '[FD] Got image'
      cv_image_gray = image_msg_to_grayscale(image_msg)
      (bbox,face_found) = faceDetect.faceDetect(cv_image_gray,classifier_xml_dir)
      print ('[FD] result ', face_found)
  
    self.result_.detected_gender = face_found
    self.action_server_.set_succeeded(self.result_)

