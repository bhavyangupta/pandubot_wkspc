#!/usr/bin/env python

import roslib
import rospy
import actionlib
import pandubot_face_detection.msg

def test_action():
  client = actionlib.SimpleActionClient("pandubot_face_detection",pandubot_face_detection.msg.face_detectAction)
  print '[FD TEST] wait for server'
  client.wait_for_server()
  print '[FD TEST] server found'
  i = 0
  result = pandubot_face_detection.msg.face_detectResult()
  i = 0
  while i < 50 and result.detected_gender == 0:
    goal = pandubot_face_detection.msg.face_detectGoal(1); # just get if the face was detected
    print '[FD TEST] send goal'
    client.send_goal(goal)
    print '[FD TEST] sent goal'
    client.wait_for_result()
    result = client.get_result()
    print result
    i = i + 1
 

if __name__ == "__main__":
  rospy.init_node("test_face_detection_server")
  # speech_out_ = SoundClient()
  print "[FD TEST] Testing face detection server"
  test_action()
  rospy.spin()