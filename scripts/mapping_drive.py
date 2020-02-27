#!/usr/bin/env python
import rospy
from lab2.msg import balboaLL # import balboa message

def parse_balboa_msg(data, self):
    # retrieve the target parameters
    self.distance = rospy.get_param("distance/target")
    self.angle = rospy.get_param("angle/target")

    # set the target parameters
    rospy.set_param("distance/target",self.distance)
    rospy.set_param("angle/target",self.angle)

class TheNode(object):
    # This class holds the rospy logic for the distance and angle target parameter 
    # based on a published balboa message and user input 

    def __init__(self):

        rospy.init_node('mapping_drive') # intialize node

        self.distance = 0 # distance variable
        self.angle = 0 # angle variable

    def main_loop(self):
        # initialize subscriber node for messages from balboa robot
        rospy.Subscriber('balboaLL', balboaLL, parse_balboa_msg, self)

        rospy.spin() # wait for messages

if __name__ == '__main__':
    try:
        a = TheNode()
        a.main_loop()
    except rospy.ROSInterruptException:
        pass
