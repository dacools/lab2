#!/usr/bin/env python
import rospy
from std_msgs.msg import Float32 # import Float32

def parse_ir_distance_msg(data, self):
    self.dist_target = rospy.get_param("distance/target")
    self.distance = data
    diff = self.target - self.distance # compute how far off the following target is

    # Determine if the difference is enough to make the robot want to move
    if diff > 5:
        self.dist_target = self.dist_target + diff
        rospy.set_param("distance/target",self.dist_target) # publish the new target
    elif diff < 5:
        self.dist_target = self.dist_target - diff
        rospy.set_param("distance/target",self.dist_target) # publish the new target

class TheNode(object):
  # This class holds the rospy logic for following behind something a set distance 

  def __init__(self):

    rospy.init_node('reactive_control') # intialize node
    self.dist_target = rospy.get_param("distance/target")
    self.distance = 0
    self.target = rospy.get_param('reactive_control_target') # init reactive control target

  def main_loop(self):
    # initialize subscriber node for messages from the ir_distance converter
    rospy.Subscriber('/ir_distance', Float32, parse_ir_distance_msg, self)

    rospy.spin() # wait for messages

if __name__ == '__main__':
    try:
        a = TheNode()
        a.main_loop()
    except rospy.ROSInterruptException:
        pass
