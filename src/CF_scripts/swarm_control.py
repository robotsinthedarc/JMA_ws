"""Created and adapted by Jakon Allred
May 2024
For use in DARC Lab 
on  a swarm of CrazyFlies"""
#!/usr/bin/env python 3

import logging
import time

import rclpy
from rclpy.node import Node

import cflib.crtp
from cflib.crazyflie.swarm import CachedCfFactory
from cflib.crazyflie.swarm import Swarm
from cflib.crazyflie.log import LogConfig

from custom_msgs import cf_cmnds, cf_data, cf_landing

class SwarmControlNode(Node):
    def __init__(self):
        super().__init__("swarm_control")

        self.get_logger().info("Swarm Control Launched")
        # set default number of drones to zero
        self.declare_parameter("ids",0)
        # get the number of drones from the launch script
        self.ids = self.get_parameter("ids").value
        self.landing = False

        # sets logging level to print errors
        logging.basicConfig(level=logging.ERROR)

        # initialize drivers
        cflib.crtp.init_drivers()

        # create subscribers to get roll, pitch, yaw, thrust commands
        # for each of the drones
        if self.ids > 0:
            self.cf01_command_subscriber = self.create_subscription(CrazyflieCommands,'/cf01/commands',self.get_commands,10)
        if self.ids > 1:
            self.cf02_command_subscriber = self.create_subscription(CrazyflieCommands,'/cf02/commands',self.get_commands,10)
        
        # subsription to the landing topic
        self.landing_sub = self.create_subscription(CrazyflieLanding,'land',self.landing_callback,10)

        # set up an initial empty vector to store ids
        # ids will be started with 10 and count by 1
        # i.e. if there are 3 drones, the ending part of the radios will be
        # 10, 11, 12
        uris = []
      
        for i in range(1, self.ids +1):
            temp_id_name = f"{i:02d}"
            new_uri = 'radio://0/80/2M/E7E7E7E7' + temp_id_name
            uris.append(new_uri)
        # set up a dictionary to store URIs
        self.uris = {
            'URI': uris
        }
        # set up a dictionary for individual URIs and send setpoint command
        self.com_dict = {}
        self.data_dict = {}
        self.data_dict1 = {}
        self.data_dict2 = {}
        self.data_dict3 = {}
        self.data_dict4 = {}
        self.data_dict5 = {}
        self.data_dict6 = {}
        # the order of the vector is [roll, pitch, yawrate, thrust]
        control_vec = [0.0, 0.0, 0.0, 0]
        # dictionary for temporarily storing drone data to be sent over a indiidual topics
        # the order is:
        # [timestamp, acc.x, acc.y, acc.z, gyro.x, gyro.y, gyro.z, x, y, z, x_des, y_des, z_des, 
        # roll, pitch, yaw, roll_des, pitch_des, m1, m2, m3, m4, voltage, current]
        data_vec1 = [0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        data_vec2 = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        data_vec3 = [0.0, 0.0, 0.0]
        data_vec4 = [0.0, 0.0, 0.0, 0.0, 0.0]
        data_vec5 = [0, 0, 0, 0, 0.0, 0.0]
        data_vec6 = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        for i in range(self.ids):
            self.com_dict[self.uris['URI'][i]]=control_vec
            self.data_dict1[self.uris['URI'][i]]=data_vec1
            self.data_dict2[self.uris['URI'][i]]=data_vec2
            self.data_dict3[self.uris['URI'][i]]=data_vec3
            self.data_dict4[self.uris['URI'][i]]=data_vec4
            self.data_dict5[self.uris['URI'][i]]=data_vec5
            self.data_dict6[self.uris['URI'][i]]=data_vec6

        #self.get_logger().info(f'{self.com_dict}')
        #self.get_logger().info(f'{self.data_dict}')

        # connect to all available drones as listed in the list of uris
        self.factory = CachedCfFactory(rw_cache='./cache')
        self.swarm = Swarm(self.uris['URI'], factory=self.factory)
        self.get_logger().info('Connecting...')
        while not self.swarm._is_open:
            # this is what keeps the link open
            self.swarm.open_links()
        self.get_logger().info('Connected to all crazyflies')

        # set up and initialize all logging configurations
        self.swarm.parallel_safe(self.init_configs)
        time.sleep(1.0)

        # set up data publishers for each of the crazyflies
        # the number of publishers set up depends on the number of crazyflies
        # that are set up in the launch file
        if self.ids > 0:
            self.cf01_publisher = self.create_publisher(
                CrazyflieData, '/cf01/data',10)
            self.create_timer(0.01,self.cf01_publish_callback)
        if self.ids > 1:
            self.cf02_publisher = self.create_publisher(
                CrazyflieData, '/cf02/data',10)
            self.create_timer(0.01,self.cf02_publish_callback)

        # arm all drones in the swarm
        self.swarm.parallel_safe(self.arming_function)
        # unlock all motors
        self.swarm.parallel_safe(self.initialize)
        # begin sending motor commands
        self.create_timer(0.01,self.motor_controller)

    def init_configs(self,scf):
        # set up logging to read IMU data
        # each LogConfig can read up to 26 bytes
        # i.e. there can be 6 floats and 1 uint16_t logged at once
        # additional configurations can be added
        # data can only be read in multiples of 10 ms
        log_imu = LogConfig(name='IMU', period_in_ms=10)
        log_imu.add_variable('acc.x','float') # x acceleration (G)
        log_imu.add_variable('acc.y','float') # y acceleration (G)
        log_imu.add_variable('acc.z','float') # z acceleration (G)
        log_imu.add_variable('gyro.x','float') # roll rate (deg/s)
        log_imu.add_variable('gyro.y','float') # pitch rate (deg/s)
        log_imu.add_variable('gyro.z','float') # yaw rate (deg/s)
        log_position = LogConfig(name='Position',period_in_ms=10)
        log_position.add_variable('stabilizer.roll', 'float') # roll (deg)
        log_position.add_variable('stabilizer.pitch', 'float') # pitch (deg)
        log_position.add_variable('stabilizer.yaw', 'float') # yaw (deg)
        log_power = LogConfig(name='Power', period_in_ms=10)
        log_power.add_variable('motor.m1','uint32_t') # motor 1 command (PWM)
        log_power.add_variable('motor.m2','uint32_t') # motor 2 command (PWM)
        log_power.add_variable('motor.m3','uint32_t') # motor 3 command (PWM)
        log_power.add_variable('motor.m4','uint32_t') # motor 4 command (PWM)
        log_power.add_variable('pm.vbat','FP16') # battery voltage (V)
        log_power.add_variable('pm.chargeCurrent','FP16') # battery current (A)
        scf.cf.log.add_config(log_imu)
        scf.cf.log.add_config(log_position)
        scf.cf.log.add_config(log_power)
        self.get_logger().info(f'Initializing logging for URI: {scf.cf.link_uri}')
        log_imu.data_received_cb.add_callback(lambda t, d, l: self.log_imu_data(scf.cf.link_uri, t, d, l))
        log_position.data_received_cb.add_callback(lambda t, d, l: self.log_position_data(scf.cf.link_uri, t, d, l))
        log_power.data_received_cb.add_callback(lambda t, d, l: self.log_power_data(scf.cf.link_uri, t, d, l))
        log_imu.start()
        log_position.start()
        log_power.start()

    def log_imu_data(self, uri, timestamp, data, logconf):
        # add the logging data into the data dictionary, keyed by URI
        data_vec = {uri: [timestamp, data['acc.x'], data['acc.y'], data['acc.z'],
                    data['gyro.x'], data['gyro.y'], data['gyro.z']]}
        self.data_dict1.update(data_vec)

    def log_position_data(self, uri, timestamp, data, logconf):
        # add the logging data into the data dictionary, keyed by URI
        data_vec = {uri: [data['stabilizer.roll'], data['stabilizer.pitch'], data['stabilizer.yaw']]}
        self.data_dict3.update(data_vec)

    def log_power_data(self, uri, timestamp, data, logconf):
        # add the logging data into the data dictionary, keyed by URI
        data_vec = {uri: [data['motor.m1'], data['motor.m2'], data['motor.m3'], data['motor.m4'],
                          data['pm.vbat'], data['pm.chargeCurrent']]}
        self.data_dict5.update(data_vec)

    def arming_function(self,scf):
        # Arm all crazyflies in the swarm
        scf.cf.platform.send_arming_request(True)
        # arm will print once for every crazyflie
        self.get_logger().info("Armed")

    def initialize(self,scf):
        # Unlock all crazyflie motors in the swarm
        scf.cf.commander.send_setpoint(0.0,0.0,0,0)
        # this will print once for every crazyflie
        self.get_logger().info("Unlocking motors")

    def motor_controller(self):
        # Calls motor commander fucntion for the swarm
        self.swarm.parallel_safe(self.motor_commander, args_dict=self.com_dict)

    def motor_commander(self,scf,des_roll,des_pitch,des_yawrate,des_thrust):
        # Sends motor commands to the swarm
        # uses the command dictionary to send individual commands
        #self.get_logger().info(f'{scf.cf.link_uri}: {des_roll:3.3f}, {des_pitch:3.3f}, {des_yawrate:3.3f}, {des_thrust}')
        scf.cf.commander.send_setpoint(des_roll,des_pitch,des_yawrate,des_thrust)
        #scf.cf.commander.send_setpoint(0.0,0.0,0,0)

    def stop_motors(self,scf):
        # Sends stop command to all crazyflies in the swarm
        # this should be changed later to a landing function
        scf.cf.commander.send_stop_setpoint()

    def get_commands(self,msg):
        # Reads the motor commands from other crazyflie 
        # command publishers into the command dictionary keyed on URI
        vec = {msg.uri: [msg.desired_roll, msg.desired_pitch, msg.desired_yaw_rate, msg.thrust]}
        if self.landing and msg.z_desired < 0.05:
            vec = {msg.uri: [0.0, 0.0, 0.0, 0]}
        self.com_dict.update(vec)
        # also reads current (x,y,z) and desired (x,y,z) position
        # which are stored in the data dictionary keyed on URI
        vec = {msg.uri: [msg.x, msg.y, msg.z, msg.x_desired, msg.y_desired, msg.z_desired]}
        self.data_dict2.update(vec)
        vec = {msg.uri: [msg.roll_mocap, msg.pitch_mocap, msg.yaw_mocap, msg.desired_roll, msg.desired_pitch]}
        self.data_dict4.update(vec)
        vec = {msg.uri: [msg.x_err, msg.x_err_dot, msg.x_err_int, 
                         msg.y_err, msg.y_err_dot, msg.y_err_int,
                         msg.z_err, msg.z_err_dot, msg.z_err_int]}
        self.data_dict6.update(vec)

    def cf01_publish_callback(self):
        # publisher for cf01 data
        # the data is sent to another node and printed to a .txt file from there
        # the node is set up in crazyflie_data_reader.py
        uri = 'radio://0/80/2M/E7E7E7E701'
        msg = CrazyflieData()
        msg.timestamp = self.data_dict1[uri][0]
        msg.acc_x = self.data_dict1[uri][1]
        msg.acc_y = self.data_dict1[uri][2]
        msg.acc_z = self.data_dict1[uri][3]
        msg.gyro_x = self.data_dict1[uri][4]
        msg.gyro_y = self.data_dict1[uri][5]
        msg.gyro_z = self.data_dict1[uri][6]
        msg.x = self.data_dict2[uri][0]
        msg.y = self.data_dict2[uri][1]
        msg.z = self.data_dict2[uri][2]
        msg.x_desired = self.data_dict2[uri][3]
        msg.y_desired = self.data_dict2[uri][4]
        msg.z_desired = self.data_dict2[uri][5]
        msg.roll_imu = self.data_dict3[uri][0]
        msg.pitch_imu = self.data_dict3[uri][1]
        msg.yaw_imu = self.data_dict3[uri][2]
        msg.roll_mocap = self.data_dict4[uri][0]
        msg.pitch_mocap = self.data_dict4[uri][1]
        msg.yaw_mocap = self.data_dict4[uri][2]
        msg.roll_desired = self.data_dict4[uri][3]
        msg.pitch_desired = self.data_dict4[uri][4]
        msg.m1 = self.data_dict5[uri][0]
        msg.m2 = self.data_dict5[uri][1]
        msg.m3 = self.data_dict5[uri][2]
        msg.m4 = self.data_dict5[uri][3]
        msg.voltage = self.data_dict5[uri][4]
        msg.current = self.data_dict5[uri][5]
        msg.x_err = self.data_dict6[uri][0]
        msg.x_err_dot = self.data_dict6[uri][1]
        msg.x_err_int = self.data_dict6[uri][2]
        msg.y_err = self.data_dict6[uri][3]
        msg.y_err_dot = self.data_dict6[uri][4]
        msg.y_err_int = self.data_dict6[uri][5]
        msg.z_err = self.data_dict6[uri][6]
        msg.z_err_dot = self.data_dict6[uri][7]
        msg.z_err_int = self.data_dict6[uri][8]

        self.cf01_publisher.publish(msg)

    def cf02_publish_callback(self):
        # publisher for cf02 data
        # the data is sent to another node and printed to a .txt file from there
        # the node is set up in crazyflie_data_reader.py
        uri = 'radio://0/80/2M/E7E7E7E702'
        msg = CrazyflieData()
        msg.timestamp = self.data_dict1[uri][0]
        msg.acc_x = self.data_dict1[uri][1]
        msg.acc_y = self.data_dict1[uri][2]
        msg.acc_z = self.data_dict1[uri][3]
        msg.gyro_x = self.data_dict1[uri][4]
        msg.gyro_y = self.data_dict1[uri][5]
        msg.gyro_z = self.data_dict1[uri][6]
        msg.x = self.data_dict2[uri][0]
        msg.y = self.data_dict2[uri][1]
        msg.z = self.data_dict2[uri][2]
        msg.x_desired = self.data_dict2[uri][3]
        msg.y_desired = self.data_dict2[uri][4]
        msg.z_desired = self.data_dict2[uri][5]
        msg.roll_imu = self.data_dict3[uri][0]
        msg.pitch_imu = self.data_dict3[uri][1]
        msg.yaw_imu = self.data_dict3[uri][2]
        msg.roll_mocap = self.data_dict4[uri][0]
        msg.pitch_mocap = self.data_dict4[uri][1]
        msg.yaw_mocap = self.data_dict4[uri][2]
        msg.roll_desired = self.data_dict4[uri][3]
        msg.pitch_desired = self.data_dict4[uri][4]
        msg.m1 = self.data_dict5[uri][0]
        msg.m2 = self.data_dict5[uri][1]
        msg.m3 = self.data_dict5[uri][2]
        msg.m4 = self.data_dict5[uri][3]
        msg.voltage = self.data_dict5[uri][4]
        msg.current = self.data_dict5[uri][5]
        msg.x_err = self.data_dict6[uri][0]
        msg.x_err_dot = self.data_dict6[uri][1]
        msg.x_err_int = self.data_dict6[uri][2]
        msg.y_err = self.data_dict6[uri][3]
        msg.y_err_dot = self.data_dict6[uri][4]
        msg.y_err_int = self.data_dict6[uri][5]
        msg.z_err = self.data_dict6[uri][6]
        msg.z_err_dot = self.data_dict6[uri][7]
        msg.z_err_int = self.data_dict6[uri][8]

        self.cf02_publisher.publish(msg)

    # new publishers should be added here as new crazyfiles are added to the swarm
    # there may be a better way to do this than setting up individual publishers
    # but I haven't found it yet

    def landing_callback(self, msg):
        self.landing = msg.land

def main(args=None):
    # Main function. Will run automatically on starting the node.
    try:
        rclpy.init(args=args)
        node = SwarmControlNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.swarm.parallel(node.stop_motors)
        node.swarm.close_links()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
