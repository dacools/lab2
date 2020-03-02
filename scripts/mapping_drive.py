#!/usr/bin/env python
import rospy
from lab2.msg import balboaLL # import balboa message
from std_msgs.msg import Bool # import bool message

PI = 3.14159265358979 # global variable for PI

def parse_balboa_msg(data, self):
    # Unpack the data needed
    ang_curr = data.angleX # unpack angle X
    ang_curr = ang_curr / 1000 # convert angle from millidegrees to degrees
    dist_curr = data.encoderCountRight # unpack right encoder
    dist_curr = dist_curr * self.DPC # convert encoder distance to mm

    # retrieve the target parameters
    self.dist = rospy.get_param("distance/target")
    self.ang = rospy.get_param("angle/target")

    if abs(dist_curr - self.dist) < 10 and abs(ang_curr - self.ang) < 2 and self.state < 13:
        step = self.seq[self.state] # get current sequence based on state

        if step == 'map1':
            self.map1(110) # map 1 cell
        elif step == 'turn':
            self.turn(90) # turn 90 degrees
        elif step == 'reverse':
            self.reverse(110) # move backwards 1 cell
        elif step == 'map2':
            self.map2(220) # map 2 cells

        self.state = self.state + 1

    # set the target parameters
    rospy.set_param("distance/target",self.dist)
    rospy.set_param("angle/target",self.ang)

class TheNode(object):
    # This class holds the rospy logic for the distance and angle target parameter 
    # based on a published balboa message and user input 

    def __init__(self):

        rospy.init_node('mapping_drive') # intialize node

        self.dist = 0 # distance variable
        self.ang = 0 # angle variable
        self.state = 0 # init a state variable
        self.mapping = false # init mapping variable
        self.line = 0 # init line count variable

        # define mapping sequence
        seq1 = ['map1','turn','reverse','map2']
        seq2 = ['reverse','turn','map2']
        self.seq = seq1 + seq2 + seq2 + seq2

        '''self.line_count = rospy.get_param('line_count') # how many lines do we want to make?
        self.turn = 1 # variable for current turn direction (turn CCW)
        self.i = 0 # variable for current line'''

        # Encoder count per revolution is gear motor ratio (3344/65)
        # times gearbox ratio (2.14/1) times encoder revolution (12/1)
        CPR = (3344 / 65) * 2.14 * 12

        # Distance per revolution is 2 PI times wheel radius (40 mm)
        distPR = 2*PI*40

        # Distance per encoder count is distPR / CPR
        self.DPC = distPR / CPR

        # initialize publisher node for map writer
        self.writer = rospy.Publisher('/writer', Bool, queue_size=10)

    def map1(self, delta):
        # update distance to map 1 cell
        self.dist = self.dist + delta

    def turn(self, delta):
        # update angle to turn
        self.ang = self.ang - delta

    def reverse(self, delta):
        # call map1 function with negative value
        self.map1(-1.0 * delta)

    def map2(self, delta):
        # call map1 function twice to map 2 cells
        self.map1(delta / 2)
        self.map1(delta / 2)

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
