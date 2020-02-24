#!/usr/bin/env python
import rospy
from lab1.msg import balboaLL # import balboa message
from lab1.msg import pid_input # import pid_input message

PI = 3.14159265358979 # global variable for PI

def parse_balboa_msg(data, self):
    self.dist_current = data.encoderCountRight # unpack right encoder
    self.dist_target = rospy.get_param('distance/target') # get distance target from user
    self.dist_current = self.dist_current * self.DPC # convert encoder distance to mm

    self.ang_current = data.angleX # unpack angle X
    self.ang_target = rospy.get_param('angle/target') # get angle target from user
    self.ang_current = self.ang_current / 1000 # convert angle from millidegrees to degrees

    # Publish the current and target distance values
    self.dist_pid_input.source = 'distance'
    self.dist_pid_input.current = self.dist_current
    self.dist_pid_input.target = self.dist_target
    self.dist.publish(self.dist_pid_input)

    # Publish the current and target angle values
    self.ang_pid_input.source = 'angle'
    self.ang_pid_input.current = self.ang_current
    self.ang_pid_input.target = self.ang_target
    self.ang.publish(self.ang_pid_input)

class TheNode(object):
  # This class holds the rospy logic for sending pid_input messages
  # from a published balboa message and user input 

  def __init__(self):

    rospy.init_node('remote_drive') # intialize node
    
    # initialize publisher node for distance PID controller
    self.dist = rospy.Publisher('/dist', pid_input, queue_size=10)

    self.dist_pid_input = pid_input() # default pid_input type
    self.dist_current = 0 # init current distance
    self.dist_target = rospy.get_param('distance/target') # init distance target

    # initialize publisher node for angle PID controller
    self.ang = rospy.Publisher('/ang', pid_input, queue_size=10)

    self.ang_pid_input = pid_input() # default pid_input type
    self.ang_current = 0 # init left angle
    self.ang_target = rospy.get_param('angle/target') # init angle target

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
