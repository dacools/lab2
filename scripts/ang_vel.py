#!/usr/bin/env python
import rospy
from lab1.msg import pid_output # import pid_output message
from lab1.msg import pid_input # import pid_output message

def parse_ang_control_msg(data, self):
    self.ang_vel_pid_input.source = 'angle_vel' # set source
    self.ang_vel_pid_input.current = 0 # set initial angle
    self.ang_vel_pid_input.target = 0 # set initial target

    if not self.ang_reached:
        # update variables when ang_vel_control is needed
        self.ang_vel_pid_input.current = data.control # set current
        self.ang_vel_pid_input.target = rospy.get_param('angle_vel/target') # set target

    # Publish the current and target angle_vel values
    self.ang_vel.publish(self.ang_vel_pid_input)

def parse_ang_msg(data, self):
    curr = data.current # unpack current angle
    tar = data.target # unpack target angle
    res = 10 # set error hysteresis 

    if (abs(tar - curr) > res):
        # angle not reached, need angle_vel
        self.ang_reached = False
    else:
        # angle reached, do not need angle_vel
        self.ang_reached = True

class TheNode(object):
  # This class holds the rospy logic for integrating the angle PID output
  # with the angle message and publishing angle_vel messages

  def __init__(self):

    rospy.init_node('ang_vel') # intialize node
    
    # initialize publisher node for angle_vel PID input messages
    self.ang_vel = rospy.Publisher('/ang_vel', pid_input, queue_size=10)
    self.ang_vel_pid_input = pid_input() # default pid_input type

    self.ang_reached = False

  def main_loop(self):
    # initialize subscriber nodes for messages from the pid controller and angle messages
    rospy.Subscriber('/ang', pid_input, parse_ang_msg, self)
    rospy.Subscriber('/ang_control', pid_output, parse_ang_control_msg, self)

    rospy.spin() # wait for messages

if __name__ == '__main__':
    try:
        a = TheNode()
        a.main_loop()
    except rospy.ROSInterruptException:
        pass
