#!/usr/bin/env python
import rospy
from lab1.msg import balboaMotorSpeeds # import motor speed message
from std_msgs.msg import Header # import header message
from geometry_msgs.msg import Twist # import Twist message
from time import sleep # import sleep function

def parse_msg(data, self):
    self.msg_data = data # unpack msg
    self.x = data.linear.x # set linear component
    self.z = data.angular.z # set angular component
    speed = 15 #set speed multiplier

    if self.x > 0:
        # move forward
        self.spdMsg.left = 1*speed
        self.spdMsg.right = 1*speed
    elif self.x < 0:
        # move backward
        self.spdMsg.left = -1*speed
        self.spdMsg.right = -1*speed
    elif self.z > 0:
        # turn left
        self.spdMsg.left = -1*speed
        self.spdMsg.right = 1*speed
    elif self.z < 0:
        # turn right
        self.spdMsg.left = 1*speed
        self.spdMsg.right = -1*speed
    else:
        rospy.loginfo("Parse error")
        
    #publish the motor speeds
    self.mtrSpd.publish(self.spdMsg)

    sleep(0.01) # delay

    # turn the motors off
    self.spdMsg.left = 0
    self.spdMsg.right = 0
    self.mtrSpd.publish(self.spdMsg)

class TheNode(object):
  # This class holds the rospy logic for sending a motor speed message
  # from a published teleop_turtle_key message

  def __init__(self):

    rospy.init_node('remote_drive') # intialize node
    
    # initialize publisher node for motorSpeeds
    self.mtrSpd = rospy.Publisher('/motorSpeeds', balboaMotorSpeeds, queue_size=10)

    self.msg_data = Twist() # Twist variable for message received
    self.x = 0 # variable for linear component
    self.z = 0 # variable for angular component
    self.spdMsg = balboaMotorSpeeds() # default motor speed msg type
    self.spdMsg.header = Header() # default header type
    self.spdMsg.left = 0 # init left speed
    self.spdMsg.right = 0 # init right speed

  def main_loop(self):
    # initialize subscriber node for messages from teleop_turtle_key
    rospy.Subscriber('/turtle1/cmd_vel', Twist, parse_msg, self)

    rospy.spin() # wait for messages

if __name__ == '__main__':
    try:
        a = TheNode()
        a.main_loop()
    except rospy.ROSInterruptException:
        pass
