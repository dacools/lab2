#!/usr/bin/env python
import rospy
from lab2.msg import balboaLL # import balboa message

PI = 3.14159265358979 # global variable for PI

def parse_balboa_msg(data, self):
    # Unpack the data needed
    self.ang_current = data.angleX # unpack angle X
    self.ang_current = self.ang_current / 1000 # convert angle from millidegrees to degrees
    self.dist_current = data.encoderCountRight # unpack right encoder
    self.dist_current = self.dist_current * self.DPC # convert encoder distance to mm

    # retrieve the target parameters
    self.distance = rospy.get_param("distance/target")
    self.angle = rospy.get_param("angle/target")
    line_distance = 300 # set the distance for a single line in the path navagation
    turn_angle = 180 # set the angle of turn
    self.turn = 1 # start by turning CCW after driving straight
    i = 0

    if abs(self.dist_current - self.distance) < 20 and abs(self.ang_current - self.angle) < 10:
        if i > self.line_count:
            pass
        elif self.state == 0: # state 0 means ready to go strait
            self.distance = self.distance + line_distance
            self.state = 1
            i = i+1
        elif self.state == 1: # state 1 means ready to turn
            if self.turn == 1: # turn 1 means turn CCW
                self.angle = self.angle + turn_angle
                self.turn = 0 # trun CW next time
            else:
                self.angle = self.angle - turn_angle
                self.turn = 1 # trun CCW next time
                
            self.state = 0 # ready to go straight again
            

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
        self.state = 0 # initialize a state variable
        self.line_count = rospy.get_param('line_count') # how many lines do we want to make?

         # Encoder count per revolution is gear motor ratio (3344/65)
        # times gearbox ratio (2.14/1) times encoder revolution (12/1)
        CPR = (3344 / 65) * 2.14 * 12

        # Distance per revolution is 2 PI times wheel radius (40 mm)
        distPR = 2*PI*40

        # Distance per encoder count is distPR / CPR
        self.DPC = distPR / CPR

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
