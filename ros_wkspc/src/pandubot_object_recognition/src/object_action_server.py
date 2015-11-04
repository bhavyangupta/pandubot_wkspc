import roslib
import rospy
import actionlib
import pandubot_object_recognition.msg
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy as np
import SIFTMatching
import cv2
import time

def image_msg_to_grayscale(image_msg):
  print 'convt to gray' 
  bridge = CvBridge()
  cv_image = bridge.imgmsg_to_cv2(image_msg,"bgr8")
  cv_image = np.array(cv_image,dtype=np.uint8)
  cv_image_gray = cv2.cvtColor(cv_image,cv2.COLOR_BGR2GRAY)
  cv_image_gray = np.array(cv_image_gray,dtype=np.uint8)
  return cv_image_gray

class ObjectAction(object):
  feedback_ = pandubot_object_recognition.msg.object_actionFeedback()
  result_ = pandubot_object_recognition.msg.object_actionResult()

  def __init__(self,name):
    print 'constructor'
    #self.detection_cb_ = detection_callback
    self._action_name = name;
    self.action_server_ = actionlib.SimpleActionServer(self._action_name,pandubot_object_recognition.msg.object_actionAction,execute_cb=self.execute_cb,auto_start = False)
    self.action_server_.start()
    print '[OR] started server'

  def execute_cb(self,goal):
    print '[OR] goal rx'
    self.feedback_.busy_code = 0    
    self.action_server_.publish_feedback(self.feedback_) # say that you will now be busy
    print '[OR] wait for image image'
    image_msg = rospy.wait_for_message("/usb_cam/image_raw",Image) # get one image and process it"
    cv_image_gray = image_msg_to_grayscale(image_msg)
    # #############333 Injections:
    # train_image_path = '/home/bhavya/pandubot_wkspc/ros_wkspc/src/pandubot_object_recognition/param/pepsi.png'
    # cv_image_gray = cv2.imread(train_image_path,0)          # queryImag
    print '[OR] got image'
    label = SIFTMatching.IsMatch(cv_image_gray)
    print ('ret val' , label)
    self.result_.detected_class = label
    self.action_server_.set_succeeded(self.result_)        


