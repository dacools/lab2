#!/usr/bin/env python
import rospy
from lab2.msg import balboaLL # import balboa message
from lab2.msg import mapPacket # import mapPacket message
import numpy as np # import numpy library

def parse_line_sensor_msg(data, self):
    self.line = data.line # unpack line number
    self.cell = data.cell # unpack cell number
    row = np.array([data.row1, data.row2, data.row3, data.row4, data.row5]) # unpack line vals
    
    if self.line < 26 and (self.cell == 2 or self.cell == 4):
        self.cell_map[25 - self.line] = row # add row to cell matrix
        rospy.set_param("map/cell",self.cell_map.tolist()) # update cell rosparam

        if self.line == 25:
            self.finished = True # cell building finished
    
    elif self.line < 26:
        self.cell_map[self.line-1] = row # add row to cell matrix
        rospy.set_param("map/cell",self.cell_map.tolist()) # update cell rosparam

        if self.line == 25:
            self.finished = True # cell building finished




    if self.finished:
        if self.cell == 1: # 1st set of rows
            self.IR_map[0:5] = np.rot90(self.cell_map,k=-1)
        elif self.cell == 2: # 2nd set of rows
            self.IR_map[5:10] = np.rot90(self.cell_map,k=3)
        elif self.cell == 3: # 3rd set of rows
            self.IR_map[10:15] = np.rot90(self.cell_map,k=1)
        # elif self.cell == 4: # 4th set of rows
        #     self.IR_map[15:20] = np.rot90(self.cell_map,k=3)
        # elif self.cell == 5: # 5th set of rows
        #     self.IR_map[20:25] = np.rot90(self.cell_map,k=1)
         

        self.finished = False # reset for a new cell
        self.cell = self.cell + 1 # Iterate the cell number to the next cell
        rospy.set_param("map/full",self.IR_map.tolist()) # update map rosparam

class TheNode(object):
    # This class holds the rospy logic for compiling line sensor results 
    # based on a published mapPacket message

    def __init__(self):
        rospy.init_node('map') # intialize node

        self.IR_map = np.zeros([15,25],dtype=int) # Init map matrix
        self.cell_map = np.zeros([25,5],dtype=int) # Init cell matrix
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
