#!/usr/bin/env python
import rospy
from lab2.msg import balboaLL # import balboa message
from lab2.msg import mapPacket # import mapPacket message

def parse_balboa_msg(data, self):
    self.threshold = rospy.get_param('threshold') # get line threshold

    # update sensor array with message data
    self.s[0] = data.sensor1
    self.s[1] = data.sensor2
    self.s[2] = data.sensor3
    self.s[3] = data.sensor4
    self.s[4] = data.sensor5

    # check if greater than threshold and update map array
    for i in range (0, 5):
        if self.s[i] > self.threshold:
            self.map[i] = 1
        else:
            self.map[i] = 0

    # update mapPacket message with map values
    self.column.row1 = self.map[0]
    self.column.row2 = self.map[1]
    self.column.row3 = self.map[2]
    self.column.row4 = self.map[3]
    self.column.row5 = self.map[4]
    self.column.i = self.column.i + 1

    # publish mapPacket
    self.mapPub.publish(self.column)

class TheNode(object):
    # This class holds the rospy logic for sending line sensor results 
    # based on a published balboa message

    def __init__(self):

        rospy.init_node('line_sensor') # intialize node

        # initialize publisher node for map packets
        self.mapPub = rospy.Publisher('/mapping', mapPacket, queue_size=10)

        self.threshold = rospy.get_param('threshold') # init line threshold
        self.map = [0, 0, 0, 0, 0] # init map array
        self.s = [0, 0, 0, 0, 0] # init sensor array
        self.column = mapPacket() # init default mapPacket message
        self.column.i = 0

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
