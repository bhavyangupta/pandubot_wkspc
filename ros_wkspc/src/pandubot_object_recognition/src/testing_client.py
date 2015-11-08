#!/usr/bin/env python

import roslib
import rospy
import actionlib
import pandubot_object_recognition.msg
import time

def test_action():

  client = actionlib.SimpleActionClient('pandubot_object_detection',pandubot_object_recognition.msg.object_actionAction)
  
  client.wait_for_server()
  while True:
    goal = 4
    goal = pandubot_object_recognition.msg.object_actionGoal(goal)
    print 'send_goal'
    client.send_goal(goal)
    print 'sent goal'
    client.wait_for_result()
    print client.get_state()
    time.sleep(1)


if __name__ == '__main__':
    rospy.init_node('object_action_test_py')
    print 'testing action'
    test_action()
