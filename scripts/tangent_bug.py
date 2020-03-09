#!/usr/bin/env python
import rospy
from lab2.msg import balboaLL # import balboa message
from std_msgs.msg import Float32 # import Float32

def parse_balboa_msg(data, self):
    # Get the current and target distances
    self.dist_current = data.encoderCountRight # unpack right encoder
    self.dist_target = rospy.get_param('distance/target') # get distance target from user
    self.dist_current = self.dist_current * self.DPC # convert encoder distance to mm
    self.dist_diff = self.target-self.dist_current # Calculate how far from the distance target we are

    # Get the current and target angles
    self.ang_current = data.angleX # unpack angle X
    self.ang_target = rospy.get_param('angle/target') # get angle target from user
    self.ang_current = self.ang_current / 1000 # convert angle from millidegrees to degrees
    self.ang_diff = self.ang_current-self.ang_target # calculate how far from the angle we are


def parse_ir_distance_msg(data, self):
    self.dist_target = rospy.get_param('distance/target') # get current distance target


    # Scanning algorithm: turn to +15 degrees, measure the distance, turn -5 degrees, measure distance, 
    # repeate until -15 degrees. select the angle with the greatest distance and the lowest angle. 
    # if the angle is less than 0, subtract 3 degrees, else add 3 degrees from the selected angle.
    # Travel to the distance -30 then scan again. Repeat

    if abs(dist_diff) < 10:
        self.scan = True 

    if self.scan:
        # Trun to +15
        if self.i == 0:
            self.ang_target = self.ang_target+15

            i = i+1








    if self.react:
        self.react = False # reset reaction
        
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
        rospy.set_param('angle/target',self.ang_target) # publish new distance target

        rospy.set_param('debug/diff',diff)
        rospy.set_param('debug/off',off)

class TheNode(object):
    # This class holds the rospy logic for navigating around an object like the tangent bug algorithm 

    def __init__(self):

        rospy.init_node('tangent_bug') # intialize node

        self.dist_target = rospy.get_param('distance/target') # init distance target
        self.angle_target = rospy.get_param('angle/target') # init distance target
        self.ir_dist = 0 # init ir distance variable

        self.ang_current = 0 # init the angle
        self.dist_current = 0 # init the distance
        self.dist_diff = 0 # init the distance diff
        self.ang_diff = 0 # init the angle diff
        self.i = 0

        self.scan = True # Tell the robot to scan

        # Initialize a matrix for the distances and angles
        self.angles_and_distances = np.matrix([[15., 0.0], [10., 0.0], [5., 0.0], [0., 0.0], [-5., 0.0], [-10., 0.0], [-15., 0.0]])




        # Encoder count per revolution is gear motor ratio (3344/65)
        # times gearbox ratio (2.14/1) times encoder revolution (12/1)
        CPR = (3344 / 65) * 2.14 * 12

        # Distance per revolution is 2 PI times wheel radius (40 mm)
        distPR = 2*PI*40

        # Distance per encoder count is distPR / CPR
        self.DPC = distPR / CPR

        

        # self.react = False # init variable to delay reaction
        # self.last_ms = 0 # init millis variable

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
