#!/usr/bin/env python
import rospy
from lab2.msg import balboaLL # import balboa message
from std_msgs.msg import Float32 # import Float32
from math import pow

def parse_balboa_msg(data, self):
    self.IR = data.IR
    # Voltage is converted to distance (cm) (Less than 15 cm is not accurate)
    self.distance = self.P1*pow(self.IR,4) + self.P2*pow(self.IR,3) +self.P3*pow(self.IR,2)+self.P4*self.IR+self.P5

    self.irPub.publish(self.distance)
    
class TheNode(object):
  # This class holds the rospy logic for receiving voltages from an IR sensor,
  # then converting the voltage to a distance and then publishing the distance 

  def __init__(self):

    rospy.init_node('ir_interpretation') # intialize node

    # initialize publisher node for IR distances 
    self.irPub = rospy.Publisher('/ir_distance', Float32, queue_size=10)
    self.IR = 0
    self.distance = Float32()

    # Curve fit parameters for a 4th order curve fit
    self.P1 = 20.4560 
    self.P2 = -159.9982  
    self.P3 = 459.6192 
    self.P4 = -597.1905  
    self.P5 = 340.1046

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
