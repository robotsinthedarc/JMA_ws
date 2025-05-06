"""Created and adapted by Jakon Allred
September 2024
For use in DARC Lab on CrazyFlie Bolt"""
#!/usr/bin/env python 3

import logging
import time
import math
import numpy as np

import rclpy
from rclpy.node import Node

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.crazyflie.log import LogConfig

from geometry_msgs.msg import PoseStamped

class CrazyflieTestNode(Node):
    """This script currently (tested):
    - reads MoCap position
    - reads IMU
    - sends motor commands
    - flies to desired z height and hover
    
    This script should be able to (untested):
    
    This script will (undeveloped):
    - fly to desired (x,y,z) position"""
    def __init__(self):
        super().__init__("crazyflie_motor_test")

        # define the radio for the crazyflie
        # the last 2 digits should be changed if connecting to a different crazyflie
        self.uri = 'radio://0/80/2M/E7E7E7E712'

        # important variables to initialize
        self.connected = False # flag for whether or not the crazyflie is connected
        self.first_command = True # flag for if the first motor command has been sent
        self.printing = False # flag to print logging variables

        # set logging level to print errors to command line
        logging.basicConfig(level=logging.ERROR)
        # initialize drivers
        cflib.crtp.init_drivers()

        # connect to the crazyflie
        self.get_logger().info("Connecting...")
        # hold until the crazyflie is connected
        while not self.connected:
            # this sets up a crazyflie object and connects to the crazyflie.
            self.scf = SyncCrazyflie(self.uri, cf=Crazyflie(rw_cache='./cache'))
            # this keeps the connection open
            self.scf.open_link()
            if self.scf.is_link_open():
                self.connected = True
                self.get_logger().info("Connected")

        # because this is a bigger crazyflie it needs to be armed in the code
        # this line arms the crazyflie because it is not done automatically
        self.scf.cf.platform.send_arming_request(True)
        self.get_logger().info("Armed")
        time.sleep(1)

        # setting the state estimator
        # defaults to complementary
        # 0 - chooses automatically
        # 1 - complementary filter
        # 2 - Extended Kalman filter
        # 3 - Unscented Kalman filter (untested, not recommended)
        self.scf.cf.param.set_value('stabilizer.estimator',1)

        # set up logging to read IMU data
        # each LogConfig can read up to 26 bytes
        # i.e. there can be 6 floats and 1 uint16_t logged at once
        # additional configurations can be added
        # data can only be read in multiples of 10 ms
        self.log_position = LogConfig(name='Position',period_in_ms=10)
        self.log_position.add_variable('stabilizer.roll', 'float') # roll (deg)
        self.log_position.add_variable('stabilizer.pitch', 'float') # pitch (deg)
        self.log_position.add_variable('stabilizer.yaw', 'float') # yaw (deg)
        self.log_power = LogConfig(name='Power', period_in_ms=10)
        self.log_power.add_variable('motor.m1','uint32_t') # motor 1 command (PWM)
        self.log_power.add_variable('motor.m2','uint32_t') # motor 2 command (PWM)
        self.log_power.add_variable('motor.m3','uint32_t') # motor 3 command (PWM)
        self.log_power.add_variable('motor.m4','uint32_t') # motor 4 command (PWM)
        self.log_power.add_variable('pm.vbat','FP16') # battery voltage (V)
        self.log_power.add_variable('pm.chargeCurrent','FP16') # battery current (A)
        self.scf.cf.log.add_config(self.log_position)
        self.scf.cf.log.add_config(self.log_power)
        self.read_logger()
        # motor control
        self.create_timer(0.01,self.motor_commander)

    def motor_commander(self):
        """sends motor commands to the crazyflie.
        send_setpoint(roll,pitch,yawrate,thrust)
        roll and pitch are in deg
        yawrate is in deg/s
        thrust is from 10000 (0%) to 60000 (100%)
        If 0 yawrate is given the crazyflie will attempt to hold 
        the current yaw, otherwise the crazyflie will spin"""
        if self.first_command:
            # sending all zeros the first time unlocks the motors
            self.scf.cf.commander.send_setpoint(0.0,0.0,0,0)
            self.first_command = False # remove the flag
            self.get_logger().info("Motors unlocked")
        else:
            self.scf.cf.commander.send_setpoint(0.0,0.0,0.0,15000)

    def read_logger(self):
        """Starts the reading of the logging data"""
        self.log_position.data_received_cb.add_callback(self.log_pos_data)
        self.log_position.start()
        self.log_power.data_received_cb.add_callback(self.log_pow_data)
        self.log_power.start()
        
    def log_pos_data(self, timestamp, data, logconf):
        if self.printing:
            self.get_logger().info(f'[{timestamp}][{logconf.name}]: ')
            for name, value in data.items():
                self.get_logger().info(f'{name}: {value:3.3f} ')
    
    def log_pow_data(self, timestamp, data, logconf):
        if self.printing:
            self.get_logger().info(f'[{timestamp}][{logconf.name}]: ')
            for name, value in data.items():
               self.get_logger().info(f'{name}: {value:3.3f} ')

def main(args=None):
    """Main function. Will run automatically on starting the node."""
    try:
        rclpy.init(args=args)
        node = CrazyflieTestNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Stopping")
        node.scf.cf.commander.send_stop_setpoint()
        node.get_logger().info("Stopped")
        node.scf.close_link()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
    