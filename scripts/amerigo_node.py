#!/usr/bin/env python3
import numpy as np
import rospy

from sensor_msgs.msg import JointState
from std_msgs.msg import String
from leap_hand_utils.dynamixel_client import *
import leap_hand_utils.leap_hand_utils as lhu
from multiprocessing import Process

class Motor:
    def __init__(self):
        ####Some parameters to control the hand
        # self.ema_amount = float(rospy.get_param('/leaphand_node/ema', '1.0')) #take only current
        self.kP = 800.0
        self.kI = 0.0
        self.kD = 200.0
        self.curr_lim = 350.0 #don't go past 600ma on this, or it'll overcurrent sometimes for regular, 350ma for lite.
        self.ema_amount = 0.2
        self.prev_pos = self.pos = self.curr_pos = lhu.amerigo(np.zeros(16))

        #You can put the correct port here or have the node auto-search for a hand at the first 3 ports.
        self.motors = motors = [0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15]
        try:
            self.dxl_client = DynamixelClient(motors, '/dev/ttyUSB0', 4000000)
            self.dxl_client.connect()
        except Exception:
            try:
                self.dxl_client = DynamixelClient(motors, '/dev/ttyUSB1', 4000000)
                self.dxl_client.connect()
            except Exception:
                self.dxl_client = DynamixelClient(motors, '/dev/ttyUSB2', 4000000)
                self.dxl_client.connect()

        #Enables position-current control mode and the default parameters, it commands a position and then caps the current so the motors don't overload
        self.dxl_client.sync_write(motors, np.ones(len(motors))*5, 11, 1)
        self.dxl_client.set_torque_enabled(motors, True)
        self.dxl_client.sync_write(motors, np.ones(len(motors)) * self.kP, 84, 2) # Pgain stiffness     
        self.dxl_client.sync_write([0,4,8], np.ones(3) * (self.kP * 0.75), 84, 2) # Pgain stiffness for side to side should be a bit less
        self.dxl_client.sync_write(motors, np.ones(len(motors)) * self.kI, 82, 2) # Igain
        self.dxl_client.sync_write(motors, np.ones(len(motors)) * self.kD, 80, 2) # Dgain damping
        self.dxl_client.sync_write([0,4,8], np.ones(3) * (self.kD * 0.75), 80, 2) # Dgain damping for side to side should be a bit less
        #Max at current (in unit 1ma) so don't overheat and grip too hard #500 normal or #350 for lite
        self.dxl_client.sync_write(motors, np.ones(len(motors)) * self.curr_lim, 102, 2)
        self.dxl_client.write_desired_pos(self.motors, self.curr_pos)

class LeapListenser:
    def __init__(self, motor):
        rospy.init_node("LeapListenser")

        self.dxl_client = motor.dxl_client
        self.motors = motor.motors
        self.prev_pos = self.pos = self.curr_pos = np.zeros(16)
        
        rospy.Subscriber("/leaphand_node/cmd", JointState, self._receive_position)

        while not rospy.is_shutdown():
            rospy.spin()

    def _receive_position(self, msg):
        position = msg.position
        self.prev_pos = self.curr_pos
        self.curr_pos = np.array(position)
        self.dxl_client.write_desired_pos(self.motors, self.curr_pos)


def main(**kwargs):
    motor = Motor()
    LeapListenser(motor)

if __name__ == "__main__":
    main()
