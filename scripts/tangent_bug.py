#!/usr/bin/env python
import rospy
from lab2.msg import balboaLL # import balboa message
from std_msgs.msg import Float32 # import Float32
import numpy as np # import numpy library

def parse_balboa_msg(data, self):
    curr_ms = data.arduinoMillis
    if curr_ms - self.last_ms > 1000:
        # one second since last reaction
        self.react = True
        self.last_ms = curr_ms

    # Get the current and target distances
    self.dist_current = data.encoderCountRight # unpack right encoder
    self.dist_goal = rospy.get_param('bug/dist_goal') # get distance goal from user
    self.dist_target = rospy.get_param('distance/target') # get distance target from user
    self.dist_current = self.dist_current * self.DPC # convert encoder distance to mm
    self.dist_diff = self.dist_goal - self.dist_current # calculate distance goal error

    # Get the current and target angles
    self.ang_current = data.angleX # unpack angle X
    self.ang_goal = rospy.get_param('bug/dist_goal') # get angle goal from user
    self.ang_target = rospy.get_param('angle/target') # get angle target from user
    self.ang_current = self.ang_current / 1000 # convert angle from millidegrees to degrees
    self.ang_diff = self.ang_goal - self.ang_current # calculate angle goal error


def parse_ir_distance_msg(data, self):
    self.dist_target = rospy.get_param('distance/target') # get current distance target

    # Scanning algorithm: turn to +15 degrees, measure the distance, turn -5 degrees, measure distance, 
    # repeate until -15 degrees. select the angle with the greatest distance and the lowest angle. 
    # if the angle is less than 0, subtract 3 degrees, else add 3 degrees from the selected angle.
    # Travel to the distance -30 then scan again. Repeat

    if abs(dist_diff) < 10:
        self.scan = True 

    if self.scan:
        # Turn to +15
        if self.i == 0:
            self.ang_target = self.ang_target + 15

            self.i = self.i + 1

        rospy.set_param('distance/target',self.dist_target) # publish new distance target
        rospy.set_param('angle/target',self.ang_target) # publish new distance target


class TheNode(object):
    # This class holds the rospy logic for navigating around an object like the tangent bug algorithm 

    def __init__(self):

        rospy.init_node('tangent_bug') # intialize node

        self.dist_goal = rospy.get_param('bug/dist_goal') # init distance goal
        self.ang_goal = rospy.get_param('bug/ang_goal') # init angle goal
        self.dist_target = rospy.get_param('distance/target') # init distance target
        self.ang_target = rospy.get_param('angle/target') # init angle target
        self.ang_current = 0 # init angle variable
        self.dist_current = 0 # init distance variable
        self.dist_diff = 0 # init distance difference variable
        self.ang_diff = 0 # init angle difference variable
        self.ir_dist = 0 # init ir distance variable
        self.scan = True # init scan variable
        self.react = False # init variable to delay reaction
        self.last_ms = 0 # init millis variable

        self.i = 0

        # Initialize a matrix for the distances and angles
        self.scan_locations = np.array([[15., 0.0], [10., 0.0], [5., 0.0], [0., 0.0], [-5., 0.0], [-10., 0.0], [-15., 0.0]])

        # Encoder count per revolution is gear motor ratio (3344/65)
        # times gearbox ratio (2.14/1) times encoder revolution (12/1)
        CPR = (3344 / 65) * 2.14 * 12

        # Distance per revolution is 2 PI times wheel radius (40 mm)
        distPR = 2*PI*40

        # Distance per encoder count is distPR / CPR
        self.DPC = distPR / CPR

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
