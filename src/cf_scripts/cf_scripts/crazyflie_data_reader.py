"""Created and adapted by Jakon Allred
May 2024
For use in DARC Lab 
For printing data to .txt file"""
#!/usr/bin/env python 3

import numpy as np
import datetime
import matplotlib as plt

import rclpy
from rclpy.node import Node

from custom_msgs.msg import CrazyflieData, CrazyflieLanding
from std_msgs.msg import Bool

class DataPrintNode(Node):
    def __init__(self):
        super().__init__("crazyflie_data_reader")

        self.landing = False
        self.start_test = False
        self.test_complete = False
        self.printed = False
        self.first_message = True

        self.declare_parameter("id",1)

        self.id = self.get_parameter("id").value
        self.formatted_id = f"{self.id:02d}"

        self.get_logger().info("cf " + self.formatted_id + " Data Reader launched")

        sub_topic = '/cf' + self.formatted_id + '/data'
        test_complete_topic = '/cf' + self.formatted_id + '/test_complete'

        self.data_sub = self.create_subscription(CrazyflieData,sub_topic,self.data_callback,10)
        self.landing_sub = self.create_subscription(CrazyflieLanding,'land',self.landing_callback,10)
        self.start_test_sub = self.create_subscription(Bool,'start_test',self.start_test_callback,10)
        self.test_complete_sub = self.create_subscription(Bool, test_complete_topic, self.test_complete_callback,10)

        self.timestamp = []
        self.acc_x = []
        self.acc_y = []
        self.acc_z = []
        self.gyro_x = []
        self.gyro_y = []
        self.gyro_z = []
        self.x = []
        self.y = []
        self.z = []
        self.x_desired = []
        self.y_desired = []
        self.z_desired = []
        self.roll_imu = []
        self.pitch_imu = []
        self.yaw_imu = []
        self.roll_mocap = []
        self.pitch_mocap = []
        self.yaw_mocap = []
        self.roll_desired = []
        self.pitch_desired = []
        self.yaw_desired = []
        self.m1 = []
        self.m2 = []
        self.m3 = []
        self.m4 = []
        self.voltage = []
        self.current = []
        self.x_err = []
        self.x_err_dot = []
        self.x_err_int = []
        self.y_err = []
        self.y_err_dot = []
        self.y_err_int = []
        self.z_err = []
        self.z_err_dot = []
        self.z_err_int = []

    def data_callback(self,msg):
        if not self.first_message and not test_complete:
        # if not self.first_message and self.start_test and not self.test_complete:
            self.timestamp.append(msg.timestamp)
            self.acc_x.append(msg.acc_x)
            self.acc_y.append(msg.acc_y)
            self.acc_z.append(msg.acc_z)
            self.gyro_x.append(msg.gyro_x)
            self.gyro_y.append(msg.gyro_y)
            self.gyro_z.append(msg.gyro_z)
            self.x.append(msg.x)
            self.y.append(msg.y)
            self.z.append(msg.z)
            self.x_desired.append(msg.x_desired)
            self.y_desired.append(msg.y_desired)
            self.z_desired.append(msg.z_desired)
            self.roll_imu.append(msg.roll_imu)
            self.pitch_imu.append(msg.pitch_imu)
            self.yaw_imu.append(msg.yaw_imu)
            self.roll_mocap.append(msg.roll_mocap)
            self.pitch_mocap.append(msg.pitch_mocap)
            self.yaw_mocap.append(msg.yaw_mocap)
            self.roll_desired.append(msg.roll_desired)
            self.pitch_desired.append(msg.pitch_desired)
            self.yaw_desired.append(msg.yaw_desired)
            self.m1.append(msg.m1)
            self.m2.append(msg.m2)
            self.m3.append(msg.m3)
            self.m4.append(msg.m4)
            self.voltage.append(msg.voltage)
            self.current.append(msg.current)
            self.x_err.append(msg.x_err)
            self.x_err_dot.append(msg.x_err_dot)
            self.x_err_int.append(msg.x_err_int)
            self.y_err.append(msg.y_err)
            self.y_err_dot.append(msg.y_err_dot)
            self.y_err_int.append(msg.y_err_int)
            self.z_err.append(msg.z_err)
            self.z_err_dot.append(msg.z_err_dot)
            self.z_err_int.append(msg.z_err_int)

        if msg.timestamp > 0 and msg.x_desired != 0:
            self.first_message = False

        if (self.landing or self.test_complete) and not self.printed:
            self.print_variables()
            self.get_logger().info('cf ' + self.formatted_id + ' variables printed to ' + datetime.datetime.now().strftime('%Y_%m_%d_%H_%M'))
            self.printed = True

    def print_variables(self):
        save_data = np.column_stack([self.timestamp, self.acc_x, self.acc_y, self.acc_z, self.gyro_x, self.gyro_y, self.gyro_z,
                                     self.x, self.y, self.z, self.x_desired, self.y_desired, self.z_desired,
                                     self.roll_imu, self.pitch_imu, self.yaw_imu, self.roll_mocap, self.pitch_mocap, self.yaw_mocap,
                                     self.roll_desired, self.pitch_desired,
                                     self.m1, self.m2, self.m3, self.m4, self.voltage, self.current,
                                     self.x_err, self.x_err_dot, self.x_err_int,
                                     self.y_err, self.y_err_dot, self.y_err_int,
                                     self.z_err, self.z_err_dot, self.z_err_int,
                                     self.yaw_desired])
        file_name = 'src/test_data/cf_' + self.formatted_id + '_data_' + datetime.datetime.now().strftime('%Y_%m_%d_%H_%M')
        np.savetxt(file_name,save_data,fmt='%1.5f')

    def landing_callback(self, msg):
        self.landing = msg.land

    def start_test_callback(self, msg):
        self.start_test = msg.data

    def test_complete_callback(self, msg):
        self.test_complete = msg.data

def main(args=None):
    try:
        rclpy.init(args=args)
        node = DataPrintNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        if not node.printed:
            node.get_logger().info('Failsafe activated: printing variables')
            node.print_variables()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()