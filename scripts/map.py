#!/usr/bin/env python
import rospy
from lab2.msg import balboaLL # import balboa message
from lab2.msg import mapPacket # import mapPacket message
import numpy as np # import numpy library

def parse_line_sensor_msg(data, self):
    self.line = data.line # unpack line number
    self.cell = data.cell # unpack cell number
    row = np.matrix([data.row1, data.row2, data.row3, data.row4, data.row5]) # unpack line vals
    
    if self.line < 6:
        self.cell_map[6-self.i] = row # add row to cell matrix

        if self.line == 5:
            self.finished = True # cell building finished

    if self.finish:
        if self.cell == 1: # Center cell
            self.IR_map[5:9][5:9] = self.cell_map
        elif self.cell == 2: # Top center cell
            self.IR_map[0:4][5:9] = np.rot90(self.cell_map)
        elif self.cell == 3: # Top left cell
            self.IR_map[0:4][0:4] = np.rot90(self.cell_map)
        elif self.cell == 4: # Middle left cell
            self.IR_map[5:9][0:4] = np.rot90(np.rot90(self.cell_map))
        elif self.cell == 5: # Bottom left cell
            self.IR_map[10:14][0:4] = np.rot90(np.rot90(self.cell_map))
        elif self.cell == 6: # Bottom center cell
            self.IR_map[10:14][5:9] = np.rot90(np.rot90(np.rot90(self.cell_map)))
        elif self.cell == 7: # Bottom right cell
            self.IR_map[10:14][10:14] = np.rot90(np.rot90(np.rot90(self.cell_map)))
        elif self.cell == 8: # Middle right cell
            self.IR_map[5:9][10:14] = self.cell_map
        elif self.cell == 9: # Top right cell
            self.IR_map[0:4][10:14] = self.cell_map  

        self.finished = False # reset for a new cell
        self.cell = self.cell + 1 # Iterate the cell number to the next cell

class TheNode(object):
    # This class holds the rospy logic for compiling line sensor results 
    # based on a published mapPacket message

    def __init__(self):
        rospy.init_node('map') # intialize node

        self.IR_map = np.zeros((25,25)) # Init map matrix
        self.cell_map = np.zeros((5,5)) # Init cell matrix
        self.cell = 1 # init cell number
        self.line = 1 # init line number
        self.finished = False # init cell building bool

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
