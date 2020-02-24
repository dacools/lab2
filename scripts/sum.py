#!/usr/bin/env python
import rospy
from lab1.msg import balboaMotorSpeeds # import motor speed message
from std_msgs.msg import Header # import header message
from lab1.msg import pid_output # import pid_output message

def parse_dist_msg(data, self):
    self.sender = data.source # unpack sender
    self.mtrspeed.left = data.control # unpack control effort
    self.mtrspeed.right = data.control # unpack control effort

    self.mtrspeed.left = self.mtrspeed.left + self.angle # sum with angle effort

    self.pub.publish(self.mtrspeed) # Publish the motor speeds

def parse_ang_vel_msg(data, self):
    self.angle = data.control # unpack control effort

class TheNode(object):
    # This class holds the rospy logic for summing the PID outputs and publishing 
    # a motor speed message

    def __init__(self):

        rospy.init_node('summation') # intialize node

        # initialize publisher node for motor speeds
        self.pub = rospy.Publisher('/motorSpeeds', balboaMotorSpeeds, queue_size=10)

        self.mtrspeed = balboaMotorSpeeds() # default motor speed msg type
        self.mtrspeed.header = Header() # default header type
        self.mtrspeed.left = 0 # init left speed
        self.mtrspeed.right = 0 # init right speed
        self.sender = '' # init sender name

        # init variable to sum distance and angle control efforts
        self.angle = 0

    def main_loop(self):
        # initialize subscriber nodes for messages from the pid controllers
        rospy.Subscriber('/ang_vel_control', pid_output, parse_ang_vel_msg, self)
        rospy.Subscriber('/dist_control', pid_output, parse_dist_msg, self)

        rospy.spin() # wait for messages

if __name__ == '__main__':
    try:
        a = TheNode()
        a.main_loop()
    except rospy.ROSInterruptException:
        pass
