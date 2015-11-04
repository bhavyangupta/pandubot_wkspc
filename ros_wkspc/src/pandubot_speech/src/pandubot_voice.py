#!/usr/bin/env python

"""
  pandubot_voice.py is the primary node that takes care of all the voice interactions.
  Based on the voice_nav.py script 
"""

import roslib; roslib.load_manifest('pandubot_voice')
import rospy

# from geometry_msgs.msg import Twist
from std_msgs.msg import String
from pandubot_voice.msg import pandu_voice_message 
from math import copysign

class pandu_voice_inter:
    def __init__(self):
        rospy.on_shutdown(self.cleanup)
        self.rate = rospy.get_param("~rate", 5)
        r = rospy.Rate(self.rate)
        self.paused = False
        
        # Initialize the Voice message we will publish.
        self.msg_location = pandu_voice_message()
        self.msg_person = pandu_voice_message()
        self.msg_object = pandu_voice_message()

        # Publish the Voice message to various topics
        self.loc_cmd_pub = rospy.Publisher('voice_location_cmd', pandu_voice_message)
        self.loc_ack_pub = rospy.Publisher('voice_location_ack', pandu_voice_message)
        self.per_cmd_pub = rospy.Publisher('voice_person_cmd', pandu_voice_message)
        self.obj_cmd_pub = rospy.Publisher('voice_object_cmd', pandu_voice_message)
        
        # Subscribe to the /recognizer/output topic to receive voice commands.
        rospy.Subscriber('/recognizer/output', String, self.speechCb)
        
        # A mapping from keywords to commands.
        self.keywords_to_command = {'navigate'        : ['go to', 'go', 'reach'],
                                    'fetch'           : ['fetch', 'get'],
                                    'vending machine' : ['vending machine'],
                                    'dustbin'         : ['dustbin'],
                                    'bench'           : ['bench'],
                                    'bump space'      : ['bump space'],
                                    'home'            : ['home'],
                                    'snickers'        : ['snickers'],
                                    'pause'           : ['pandu listen','listen'],
                                    'continue'        : ['pandu dont listen', 'do not listen','not listen']}
        
        rospy.loginfo("Ready to receive voice commands")
        
        # We have to keep publishing the cmd_vel message if we want the robot to keep moving.
        while not rospy.is_shutdown():
           self.loc_cmd_pub.publish(self.msg_location)
           self.per_cmd_pub.publish(self.msg_person)
           self.obj_cmd_pub.publish(self.msg_object)
           r.sleep()                       
            
    def get_command(self, data):
        for (command, keywords) in self.keywords_to_command.iteritems():
            for word in keywords:
                if data.find(word) > -1:
                    return command
        
    def speechCb(self, msg):        
        command = self.get_command(msg.data)
        
        rospy.loginfo("Command: " + str(command))
        
        if command == 'pause':
            self.paused = True
        elif command == 'continue':
            self.paused = False
            
        if self.paused:
            return       

        self.msg_location.first_string = 'none'
        self.msg_location.second_string = 'none'
        self.msg_person.first_string = 'none'
        self.msg_person.second_string = 'none'
        self.msg_object.first_string = 'none'
        self.msg_object.second_string = 'none'


#Set of actions
        if command == 'navigate':    
            self.msg_location.second_string = 'navigate'
            
        elif command == 'fetch':
            self.msg_object.second_string = 'fetch'
                
#Set of Locations
        if command == 'vending machine':  
            self.msg_location.first_string = 'vending machine'      
            
        elif command == 'bump space':
             self.msg_location.first_string = 'bump space'   
    
        elif command == 'bench':    
             self.msg_location.first_string = 'bench'           
                
        elif command == 'dustbin':
             self.msg_location.first_string = 'dustbin'   
            
        elif command == 'home': 
             self.msg_location.first_string = 'home'
   
#Set of Objects
        if command == 'snickers':
             self.msg_object.first_string = 'snickers'   
        
        else:
            return



   
if __name__=="__main__":
    rospy.init_node('pandubot_voice')
    try:
        pandu_voice_inter()
    except:
        pass

