#!/usr/bin/env python
import rospy
from lab1.msg import pid_input # import pid_input message
from lab1.msg import pid_output # import pid_output message

def controller(data, self):
    source = data.source # unpack message source

    # get most recent rosparam values
    self.P = rospy.get_param('{0}/P'.format(source))
    self.I = rospy.get_param('{0}/I'.format(source))
    self.D = rospy.get_param('{0}/D'.format(source))

    curr = data.current # unpack current value
    tar = data.target # unpack target value

    # err is (target - current)
    err = tar - curr

    # err_P term is (P*err)
    err_P = self.P*err

    # err_I term is (I*(err_sum + err)
    err_I = self.I*(self.err_sum + err)

    # err_D term is (D*(err - err_last))
    err_D = self.D*(err - self.err_last)

    self.output.source = source # set source
    self.output.control = err_P + err_I + err_D # set control effort
    self.target.publish(self.output) # publish output msg

    rospy.loginfo(self.output) # debug

    # err_last is (previous_target - previous_current)
    self.err_last = err

    # err_sum is accumulated err plus current err
    self.err_sum = self.err_sum + err

class TheNode(object):
    # This class holds the rospy logic for a generic PID controller

    def __init__(self):

        rospy.init_node('pid', anonymous=True) # intialize node

        # initialize publisher node for generic PID controller
        self.target = rospy.Publisher('/control', pid_output, queue_size=10)

        self.output = pid_output()  # default pid_output type
        self.err_last = 0 # init derivative term
        self.err_sum = 0 # init integral term

    def main_loop(self):
        # initialize subscriber node for messages from a generic source 
        rospy.Subscriber('/subscribe', pid_input, controller, self)

        rospy.spin() # wait for messages

if __name__ == '__main__':
    try:
        a = TheNode()
        a.main_loop()
    except rospy.ROSInterruptException:
        pass
