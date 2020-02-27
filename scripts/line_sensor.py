#!/usr/bin/env python
import rospy
from lab2.msg import balboaLL # import balboa message

def parse_balboa_msg(data, self):
    self.s[0] = data.sensor1
    self.s[1] = data.sensor2
    self.s[2] = data.sensor3
    self.s[3] = data.sensor4
    self.s[4] = data.sensor5

    for i in range (0, 5):
        if self.s[i] > self.threshold:
            self.line[i] = 1
        else:
            self.line[i] = 0

class TheNode(object):
  # This class holds the rospy logic for sending line sensor results 
  # based on a published balboa message and user input 

  def __init__(self):

    rospy.init_node('line_sensor') # intialize node
    
    # initialize publisher node for distance PID controller
    self.line_reading = rospy.Publisher('/dist', pid_input, queue_size=10)

    self.threshold = rospy.get_param('threshold') # init distance target
    self.line = [0, 0, 0, 0, 0]
    self.s = [0, 0, 0, 0, 0]
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