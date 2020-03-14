#!/usr/bin/env python
import rospy
from lab2.msg import balboaLL # import balboa message
from std_msgs.msg import Float32 # import Float32
import numpy as np # import numpy library
import math # import math library

PI = 3.14159265358979 # global variable for PI

def parse_balboa_msg(data, self):
    curr_ms = data.arduinoMillis

    # Get the current and target distances
    self.dist_current = data.encoderCountRight # unpack right encoder
    self.dist_target = rospy.get_param('distance/target') # get distance target from user
    self.dist_current = self.dist_current * self.DPC # convert encoder distance to mm
    self.dist_diff = self.dist_target - self.dist_current # calculate distance goal error

    # Get the current and target angles
    self.ang_current = data.angleX # unpack angle X
    self.ang_target = rospy.get_param('angle/target') # get angle target from user
    self.ang_current = self.ang_current / 1000 # convert angle from millidegrees to degrees
    self.ang_diff = self.ang_target - self.ang_current # calculate angle goal error

def parse_ir_distance_msg(data, self):
    self.dist_target = rospy.get_param('distance/target') # get current distance target
    self.ang_target = rospy.get_param('angle/target') # get angle target from user
    self.ir_dist = data.data*10 # convert ir distance to mm
    if self.ir_dist > 500:
        self.ir_dist = 500

    # Scanning algorithm: 
    # turn to +15 degrees, measure the distance, turn -5 degrees, measure distance, 
    # repeate until -15 degrees. select the angle with the greatest distance and the lowest angle. 
    # add some angle to the lowest angle so that it avoids the object.
    # Travel to the distance then scan again. Repeat
    # if there is nothing measured within a distance of greater than 50, Travel 50 toward the goal, 
    # then measure the objects again. 

    if abs(self.dist_diff) <= 10 and abs(self.ang_diff) <= 1:
        self.scan = True 
    else:
        self.scan = False

    if self.scan:
        # Turn to +15
        if self.state == 'turn_right_to_start_scan':
            self.ang_target = 15.0 # Turn to the right 15 degrees
            self.state = 'turn_left_x_degrees'

        # Turn to -5 degrees until all the measurments have been read
        elif self.state == 'turn_left_x_degrees':
            self.scan_locations[self.i][1] = float(self.ir_dist) # Measure the ir distance
            if self.i == 6:
                self.state = 'turn_to_best_angle'
                self.i = 0 # reset i
            else:
                self.ang_target = float(self.scan_locations[self.i+1][0]) # Turn to the left 5 degrees
                self.i = self.i + 1 # increment i

        elif self.state == 'turn_to_best_angle':
            for x in range(0,7):
                # Find the best distance
                if float(self.scan_locations[x][1]) > self.best_dist: 
                    self.best_dist = float(self.scan_locations[x][1])
                    self.best_angle = float(self.scan_locations[x][0])
                # if there are more than one best distance, find the best angle
                elif float(self.scan_locations[x][1]) == self.best_dist:
                    if abs(float(self.scan_locations[x][0])) < self.best_angle:
                        self.best_angle = float(self.scan_locations[x][0])

            # calculate how much to change the angle in order to get around the object
            self.avoid_object = math.degrees(math.atan(120/self.best_dist))

            # Check to see if the best angle is to the left or right of the object
            if self.best_angle >= 0:
                self.ang_target = self.best_angle + self.avoid_object  # Turn to the best angle avoiding the object
                self.state = 'move_to_best_dist'
            else:
                self.ang_target = self.best_angle - self.avoid_object  # Turn to the best angle avoiding the object
                self.state = 'move_to_best_dist'
        
        # move to the new target distance
        elif self.state == 'move_to_best_dist':
            self.dist_target = self.dist_target - self.best_dist - 50 # move to the best distance + 5 cm
            self.state = 'turn_right_to_start_scan'
            self.best_angle = 0.0
            self.best_dist = 0.0            

        rospy.set_param('distance/target',self.dist_target) # publish new distance target
        rospy.set_param('angle/target',self.ang_target) # publish new distance target
        self.scan = False # reset the scanner

class TheNode(object):
    # This class holds the rospy logic for navigating around an object like the tangent bug algorithm 

    def __init__(self):

        rospy.init_node('tangent_bug') # intialize node

        self.dist_target = rospy.get_param('distance/target') # init distance target
        self.ang_target = rospy.get_param('angle/target') # init angle target
        self.ang_current = 0.0 # init angle variable
        self.dist_current = 0.0 # init distance variable
        self.dist_diff = 0.0 # init distance difference variable
        self.ang_diff = 0.0 # init angle difference variable
        self.ir_dist = 0.0 # init ir distance variable
        self.scan = True # init scan variable
        self.best_dist = 1 # init the best distance to travel
        self.best_angle = 0.0 # init the best angle
        self.avoid_object = 0.0 # init the extra angle to be added to the best angle to avoid objects
        self.state = 'turn_right_to_start_scan' # init the state to turn to the right x degrees
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
