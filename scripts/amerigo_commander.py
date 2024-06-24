#!/usr/bin/env python3
import numpy as np
import rospy

from sensor_msgs.msg import JointState
from std_msgs.msg import String
from leap_hand_utils.dynamixel_client import *
import leap_hand_utils.leap_hand_utils as lhu
from multiprocessing import Process

class AmerigoCommander:
    def __init__(self):
        rospy.init_node("AmerigoCommander")

        self.pos_command = np.zeros(16)
        self.publisher = rospy.Publisher('/leaphand_node/cmd', JointState, queue_size=10)

        while not rospy.is_shutdown():
            command = input("Please input state. 0: open fingers, 1: ready_pose, 2: close fingers.")
            if(str(command) == "0"):
                self.pos_command = np.zeros(16)
            elif(str(command) == "1"):
                self.pos_command = [0.0, 0.0, 0.0, 0.0,
                                    0.0, 0.0, 0.0, 0.0,
                                    0.0, 0.0, 0.0, 0.0,
                                    1.57, -1.57, 0.0, 0.0]
            elif(str(command) == "2"):
                self.pos_command = [-0.8, 0.0, 0.8, -0.8,
                                    -0.8, 0.0, 0.8, -0.8,
                                    -0.8, 0.0, 0.8, -0.8,
                                    1.57, -1.57, 0.6, 0.6]
            msg = JointState()
            msg.header.frame_id = "amerigo"
            msg.header.stamp = rospy.Time.now()
            msg.position = self.pos_command
            msg.position = self.pos_command
            self.publisher.publish(msg)

def main(**kwargs):
    AmerigoCommander()

if __name__ == "__main__":
    main()
