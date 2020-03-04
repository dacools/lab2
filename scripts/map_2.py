#!/usr/bin/env python
import rospy
from lab2.msg import balboaLL # import balboa message
from lab2.msg import mapPacket # import mapPacket message
import numpy as np # import numpy library

def parse_line_sensor_msg(data, self):
    self.line = data.line # unpack line number
    self.cell = data.cell # unpack cell number
    row = np.array([data.row1, data.row2, data.row3, data.row4, data.row5]) # unpack line vals
    
    if self.line < 6:
        self.cell_map[5-self.line] = row # add row to cell matrix
        rospy.set_param("map/cell",self.cell_map.tolist()) # update cell rosparam

        if self.line == 5:
            self.finished = True # cell building finished

    if self.finished:
        if self.cell == 1: # Center cell
            self.IR_map[5:10,5:10] = self.cell_map
        elif self.cell == 2: # Top center cell
            self.IR_map[0:5,5:10] = np.rot90(self.cell_map,k=3)
        elif self.cell == 3: # Top right cell
            self.IR_map[0:5,10:15] = np.rot90(self.cell_map,k=3)
        elif self.cell == 4: # Middle right cell
            self.IR_map[5:10,10:15] = np.rot90(self.cell_map,k=2)
        elif self.cell == 5: # Bottom right cell
            self.IR_map[10:15,10:15] = np.rot90(self.cell_map,k=1)
        elif self.cell == 6: # Bottom center cell
            self.IR_map[10:15,5:10] = np.rot90(self.cell_map,k=1)
        elif self.cell == 7: # Bottom left cell
            self.IR_map[10:15,0:5] = np.rot90(self.cell_map,k=1)
        elif self.cell == 8: # Middle left cell
            self.IR_map[5:10,0:5] = self.cell_map
        elif self.cell == 9: # Top left cell
            self.IR_map[0:5,0:5] = self.cell_map  

        self.finished = False # reset for a new cell
        self.cell = self.cell + 1 # Iterate the cell number to the next cell
        rospy.set_param("map/full",self.IR_map.tolist()) # update map rosparam

class TheNode(object):
    # This class holds the rospy logic for compiling line sensor results 
    # based on a published mapPacket message

    def __init__(self):
        rospy.init_node('map') # intialize node

        self.IR_map = np.zeros([15,15],dtype=int) # Init map matrix
        self.cell_map = np.zeros([5,5],dtype=int) # Init cell matrix
        self.cell = 1 # init cell number
        self.line = 1 # init line number
        self.finished = False # init cell building bool

        rospy.set_param("map/full",self.IR_map.tolist()) # init map rosparam
        rospy.set_param("map/cell",self.cell_map.tolist()) # init cell rosparam

    def main_loop(self):
        # initialize subscriber node to receive mapping information
        rospy.Subscriber('/mapping', mapPacket, parse_line_sensor_msg, self)

        rospy.spin() # wait for messages

if __name__ == '__main__':
    try:
        a = TheNode()
        a.main_loop()
    except rospy.ROSInterruptException:
        pass
