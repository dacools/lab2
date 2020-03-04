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

    if abs(dist_curr - self.dist) < 10 and abs(ang_curr - self.ang) < 2 and self.state < 37:
        width = 65 # IR sensor width is 65 mm, wheel base width is 110 mm

        # target location reached
        if self.mapping and self.send:
            self.writer.publish(True) # send line message
            self.send = False
        elif self.mapping and self.line < 6:
            self.move(width/5) # move to next line
            self.line = self.line + 1 # update line counter
            self.send = True # send values
        else:        
            step = self.seq[self.state] # get current sequence based on state

            if step == 'map1':
                self.mapping = True # map 1 cell
                self.line = 1
            elif step == 'turn':
                self.mapping = False
                self.turn(90) # turn 90 degrees
            elif step == 'reverseR':
                self.mapping = False
                self.reverse(50) # move backwards for right side
            elif step == 'reverseL':
                self.mapping = False
                self.reverse(180) # move backwards for left side

            self.state = self.state + 1

        # set the target parameters
        rospy.set_param("distance/target",self.dist)
        rospy.set_param("angle/target",self.ang)

    elif self.state == 37: # Show that we are done mapping
        self.move(200)
        self.state = self.state + 1

        # set the target parameters
        rospy.set_param("distance/target",self.dist)

    rospy.set_param("debug/mapping",self.mapping)
    rospy.set_param("debug/send",self.send)
    rospy.set_param("debug/line",self.line)
    rospy.set_param("debug/state",self.state)
    rospy.set_param("debug/step",self.seq[self.state - 1])

class TheNode(object):
    # This class holds the rospy logic for updating the distance and angle target parameters 
    # based on a published balboa message and user input 

    def __init__(self):

        rospy.init_node('mapping_drive') # intialize node

        self.dist = 0 # distance variable
        self.ang = 0 # angle variable
        self.state = 0 # init a state variable
        self.mapping = False # init mapping variable
        self.send = False # init sending variable
        self.line = 0 # init line count variable

        # define mapping sequence
        seq1 = ['map1','map1','map1','map1','map1']
        seq2 = ['turn','reverseR','turn']
        seq3 = ['turn','reverseL','turn']
        self.seq = seq1 + seq2 + seq1 + seq3 + seq1 + seq2 + seq1 + seq3 + seq1

        # Encoder count per revolution is gear motor ratio (3344/65)
        # times gearbox ratio (2.14/1) times encoder revolution (12/1)
        CPR = (3344 / 65) * 2.14 * 12

        # Distance per revolution is 2 PI times wheel radius (40 mm)
        distPR = 2*PI*40

        # Distance per encoder count is distPR / CPR
        self.DPC = distPR / CPR

        # initialize publisher node for map writer
        self.writer = rospy.Publisher('/line_queue', Bool, queue_size=10)

    def move(self, delta):
        # update distance target
        self.dist = self.dist + delta

    def turn(self, delta):
        # update angle target
        self.ang = self.ang - delta

    def reverse(self, delta):
        # call move function with negative value
        self.move(-1.0 * delta)

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
