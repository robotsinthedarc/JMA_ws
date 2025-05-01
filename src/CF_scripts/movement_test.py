"""Created and adapted by Jakon Allred
April 2024
For use in DARC Lab on CrazyFlie Bolt"""
#!/usr/bin/env python 3

import logging
import time
import math

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
    - moe to a new desired position after 10 seconds"""
    def __init__(self):
        super().__init__("movment_test")

        # define the radio for the crazyflie
        # the last 2 digits should be changed if connecting to a different crazyflie
        self.uri = 'radio://0/80/2M/E7E7E7E710'

        # important variables to initialize
        self.connected = False # flag for whether or not the crazyflie is connected
        self.first_command = True # flag for if the first motor command has been sent
        self.first_position = True # flag for reading the initial position
        self.change_desired = False

        self.z_desired = 0.3 # desired height in meters
        self.roll_des = 0.0
        self.pitch_des = 0.0

        self.z_err = 0.0 # initial error
        self.z_err_prev = 0.0 # initial previous error
        self.z_err_dot = 0.0 # initial error derivative
        self.thrust = 0

        self.y_err = 0.0
        self.y_err_prev = 0.0
        self.y_err_dot = 0.0

        self.x_err = 0.0
        self.x_err_prev = 0.0
        self.x_err_dot = 0.0

        # set controller gains
        self.kp_z = 90.0
        self.kd_z = 18.0
        self.kp_xy = 12.0
        self.kd_xy = 15.0
        
        self.omega = 5.0 / 10.0

        # set logging level to print errors to command line
        logging.basicConfig(level=logging.ERROR)
        # initialize drivers
        cflib.crtp.init_drivers()

        # create MoCap subscriber for position/orientation
        self.position_sub = self.create_subscription(PoseStamped,'/fly10/pose',self.position_callback,10)

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

        # setting the state estimator
        # defaults to complementary
        # 0 - chooses automatically
        # 1 - complementary filter
        # 2 - Extended Kalman filter
        # 3 - Unscented Kalman filter (untested, not recommended)
        self.scf.cf.param.set_value('stabilizer.estimator',1)

        # set up logging to read IMU data
        self.log_attitude = LogConfig(name='Stabilizer', period_in_ms=10)
        self.log_attitude.add_variable('stabilizer.roll', 'float')
        self.log_attitude.add_variable('stabilizer.pitch', 'float')
        self.log_attitude.add_variable('stabilizer.yaw', 'float')
        self.scf.cf.log.add_config(self.log_attitude)
        # comment the following line to suppress logger
        #self.create_timer(0.1,self.read_logger)

        # motor control
        self.create_timer(0.01,self.motor_commander)

        # set up timers to read elapsed time
        self.init_time = time.time()
        self.t = time.time() - self.init_time
        self.t_prev = self.t

    def position_callback(self,msg):
        """reads the position and orientation from MoCap
        also sends position and orientation to the crazyflie"""

        # read the (x,y,z) position of the crazyflie from MoCap
        self.x = msg.pose.position.x
        self.y = msg.pose.position.y
        self.z = msg.pose.position.z
        # read the (qx,qy,qz,qw) quaternion for orientation of the crazyflie from MoCap
        self.qx = msg.pose.orientation.x
        self.qy = msg.pose.orientation.y
        self.qz = msg.pose.orientation.z
        self.qw = msg.pose.orientation.w

        # send the MoCap position and orientation to the crazyflie
        # probably not necessary
        #self.scf.cf.extpos.send_extpose(self.x,self.y,self.z,self.qx,self.qy,self.qz,self.qw)

        # convert quaternions to euler angles
        self.euler_from_quaternion()
        #self.get_logger().info('x = %s, y = %s z = %s' %(self.x, self.y, self.z))
        #self.get_logger().info(f'roll = {self.roll:3.3f}, pitch = {self.pitch:3.3f}, yaw = {self.yaw:3.3f}')

        # read and set initial (x,y,z) and yaw values
        if self.first_position:
            self.first_position = False
            self.x0 = msg.pose.position.x
            self.y0 = msg.pose.position.y
            self.z0 = msg.pose.position.z
            self.yaw0 = self.yaw
            self.x_desired = self.x0
            self.y_desired = self.y0

    def euler_from_quaternion(self):
        """takes the quaternions from MoCap and converts them to Euler.
        Way easier to visualize and verify"""

        t0 = +2.0 * (self.qw * self.qx + self.qy * self.qx)
        t1 = +1.0 - 2.0 * (self.qx * self.qx + self.qy * self.qy)
        self.roll = math.atan2(t0,t1) * 180.0 / math.pi
        t2 = +2.0 * (self.qw * self.qy - self.qz * self.qx)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        self.pitch = math.asin(t2) * 180.0 / math.pi
        t3 = +2.0 * (self.qw * self.qz + self.qx * self.qy)
        t4 = +1.0 - 2.0 * (self.qy * self.qy + self.qz * self.qz)
        self.yaw = math.atan2(t3, t4) * 180.0 / math.pi

    def motor_commander(self):
        """sends motor commands to the crazyflie.
        send_setpoint(roll,pitch,yawrate,thrust)
        roll and pitch are in deg
        yawrate is in deg/s
        thrust is from 10000 (0%) to 60000 (100%)
        If 0 yawrate is given the crazyflie will attempt to hold 
        the current yaw, otherwise the crazyflie will spin"""

        if not self.first_position:
            if self.first_command:
                # sending all zeros the first time unlocks the motors
                self.scf.cf.commander.send_setpoint(0.0,0.0,0,0)
                self.first_command = False # remove the flag
                self.get_logger().info("Unlocking motors")
            else:
                self.motor_controller()
                self.scf.cf.commander.send_setpoint(self.roll_des,self.pitch_des,0.0,self.thrust)
                # uncomment following lines for debugging
                #self.get_logger().info(f'thrust = {self.thrust}')
                #self.get_logger().info(f'roll_des = {self.roll_des:3.3f}')
                #self.get_logger().info(f'pitch_des = {self.pitch_des:3.3f}')

    def read_logger(self):
        """Starts the reading of the logging data"""

        self.log_attitude.data_received_cb.add_callback(self.log_data)
        self.log_attitude.start()

    def log_data(self, timestamp, data, logconf):
        """Prints the logging data"""

        self.get_logger().info(f'[{timestamp}][{logconf.name}]: ')
        for name, value in data.items():
            self.get_logger().info(f'{name}: {value:3.3f} ')

    def motor_controller(self):
        """Function for calculating necessary motor thrust
        Eventually pitch and roll will be calculated too"""

        # calculate elapsed time since last time through the loop
        self.t_prev = self.t
        self.t = time.time() - self.init_time
        delta_t = self.t - self.t_prev

        if self.t > 5.0:
            self.x_desired = self.x0 + 0.5 * (1.0 - math.cos(self.omega * (self.t - 10.0)))
            self.y_desired = self.y0 + 0.5 * math.sin(self.omega * (self.t - 10.0))

        self.get_logger().info(f'x_des={self.x_desired:3.3f}, y_des={self.y_desired:3.3f}')

        # calculate z error and de_z/dt
        self.z_err_prev = self.z_err
        self.z_err = self.z_desired - self.z
        self.z_err_dot = (self.z_err - self.z_err_prev) / delta_t

        # calculate y error and de_y/dt
        self.y_err_prev = self.y_err
        self.y_err =  -(self.y_desired - self.y)
        self.y_err_dot = (self.y_err - self.y_err_prev) / delta_t

        # calculate x error and de_x/dt
        self.x_err_prev = self.x_err
        self.x_err = self.x_desired - self.x
        self.x_err_dot = (self.x_err - self.x_err_prev) / delta_t

        # calculate thrust needed
        # need to map force value to thrust value
        # thrust ranges from 10000 to 60000
        force = self.kp_z * self.z_err + self.kd_z * self.z_err_dot
        self.thrust = math.floor(25000 + 1000 * force)
        if self.thrust > 60000: # higher than max, cap the thrust
            self.thrust = 60000
        elif self.thrust < 10000: # lower than min, set to min
            self.thrust = 10000

        # calculate desired roll and pitch
        self.roll_des = self.kp_xy * self.y_err + self.kd_xy * self.y_err_dot
        self.pitch_des = self.kp_xy * self.x_err + self.kd_xy * self.x_err_dot
        if self.roll_des > 45.0:
            self.roll_des = 45.0
        elif self.roll_des < -45.0:
            self.roll_des = -45.0
        if self.pitch_des > 45.0:
            self.pitch_des = 45.0
        elif self.pitch_des < -45.0:
            self.pitch_des = -45.0

def main(args=None):
    """Main function. Will run automatically on starting the node."""
    try:
        rclpy.init(args=args)
        node = CrazyflieTestNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Stopping")
        node.scf.cf.commander.send_stop_setpoint()
        #node.get_logger().info("Stopped")
        node.scf.cf.platform.send_arming_request(False)
        node.scf.close_link()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
    