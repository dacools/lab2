#!/usr/bin/env python
import rospy
from lab2.msg import balboaLL # import balboa message
from lab2.msg import mapPacket # import mapPacket message
import numpy as np

def parse_line_sensor_msg(data, self):
    self.line = data.line
    self.cell_number = data.cell
    row = np.matrix([data.row1, data.row2, data.row3, data.row4, data.row5])
    
    if self.line < 6:
        self.cell_map[6-self.i] = row

        if self.line = 5:
            self.finish = True

    if self.finish:
        if self.cell_number = 1: # Center cell
            self.IR_map[5:9][5:9] = self.cell_map
        elif self.cell_number = 2: # Top center cell
            self.IR_map[0:4][5:9] = np.rot90(self.cell_map)
        elif self.cell_number = 3: # Top left cell
            self.IR_map[0:4][0:4] = np.rot90(self.cell_map)
        elif self.cell_number = 4: # Middle left cell
            self.IR_map[5:9][0:4] = np.rot90(np.rot90(self.cell_map))
        elif self.cell_number = 5: # Bottom left cell
            self.IR_map[10:14][0:4] = np.rot90(np.rot90(self.cell_map))
        elif self.cell_number = 6: # Bottom center cell
            self.IR_map[10:14][5:9] = np.rot90(np.rot90(np.rot90(self.cell_map)))
        elif self.cell_number = 7: # Bottom right cell
            self.IR_map[10:14][10:14] = np.rot90(np.rot90(np.rot90(self.cell_map)))
        elif self.cell_number = 8: # Middle right cell
            self.IR_map[5:9][10:14] = self.cell_map
        elif self.cell_number = 9: # Top right cell
            self.IR_map[0:4][10:14] = self.cell_map  

        self.finish = False # reset for a new cell
        self.cell_number = self.cell_number + 1 # Iterate the cell number to the next cell

class TheNode(object):
    # This class holds the rospy logic for sending line sensor results 
    # based on a published balboa message

    def __init__(self):
        rospy.init_node('map') # intialize node

        # Initialize the map matrix
        self.IR_map = np.zeros((25,25))
        self.cell_map = np.zeros((5,5))
        self.cell_number = 1
        self.line = 1
        self.finish = False

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
