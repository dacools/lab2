#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist # import Twist message

class TheNode(object):
  # This class holds the rospy logic for sending an angle or distance multiplier

  def __init__(self):
    rospy.init_node('key_board_input') # intialize node
    
    # initialize publisher node for key_input
    self.pub = rospy.Publisher('/key_input', Twist, queue_size=10)

    self.key = Twist()
    self.key.linear.y = 1
    self.key.angular.y = 1

  def main_loop(self):
    # define input prompt string
    inputStr = "Enter '1' through '5' to change distance multiplier\n"
    inputStr = inputStr + "Or enter '6' through '10' to change the angle multiplier\n"
    inputStr = inputStr + "Press 'Enter' to send the multiplier to balboa\n"

    while not rospy.is_shutdown(): # wait for an input from the user
      abc = input(inputStr)

      if abc < 6:
          # change the distance multiplier
          self.key.linear.y = int(abc)
      elif abc < 11:
          # change the angle multiplier 
          self.key.angular.y = int(abc)-5
      else:
          print 'Input not valid, try again.'

      self.pub.publish(self.key)

if __name__ == '__main__':
    try:
        a = TheNode()
        a.main_loop()
    except rospy.ROSInterruptException:
        pass
