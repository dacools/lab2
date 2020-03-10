#!/usr/bin/env python
import rospy
from lab2.msg import balboaLL # import balboa message
from std_msgs.msg import Float32 # import Float32

def parse_balboa_msg(data, self):
    curr_ms = data.arduinoMillis
    if curr_ms - self.last_ms > 100:
        # one-tenth second since last reaction
        self.react = True
        self.last_ms = curr_ms

def parse_ir_distance_msg(data, self):
    if self.react:
        self.react = False # reset reaction
        self.dist_target = rospy.get_param('distance/target') # get current distance target
        self.ir_target = rospy.get_param('reactive/target') # get current reactive target
        self.ir_dist = data.data # get current ir distance
        diff = self.ir_target - self.ir_dist # compute ir target error
        off = False # debugging

        if self.ir_dist < 15:
            # out of calibration range, do nothing
            off = True
        elif self.ir_dist > 60:
            # out of useful range, do nothing
            off = True
        elif diff > 2:
            self.dist_target = self.dist_target + 5 # add offset to distance target
        elif diff < -2:
            self.dist_target = self.dist_target - 5 # subtract offset to distance target

        rospy.set_param('distance/target',self.dist_target) # publish new distance target

        rospy.set_param('debug/diff',diff)
        rospy.set_param('debug/off',off)

class TheNode(object):
    # This class holds the rospy logic for following at a set distance 

    def __init__(self):

        rospy.init_node('reactive_control') # intialize node

        self.dist_target = rospy.get_param('distance/target') # init distance target
        self.ir_target = rospy.get_param('reactive/target') # init reactive target
        self.ir_dist = 0 # init ir distance variable
        self.react = False # init variable to delay reaction
        self.last_ms = 0 # init millis variable

    def main_loop(self):
        # initialize subscriber node for messages from the ir_distance converter
        rospy.Subscriber('balboaLL', balboaLL, parse_balboa_msg, self)
        rospy.Subscriber('/ir_distance', Float32, parse_ir_distance_msg, self)

        rospy.spin() # wait for messages

if __name__ == '__main__':
    try:
        a = TheNode()
        a.main_loop()
    except rospy.ROSInterruptException:
        pass
