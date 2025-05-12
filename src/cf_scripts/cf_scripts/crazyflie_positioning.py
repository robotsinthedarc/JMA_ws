"""Created and adapted by Jakon Allred
May 2024
For use in DARC Lab 
For publishing desired positions
to individual Crazyflies"""
#!/usr/bin/env python 3

import math
import time
import numpy as np

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped
from custom_msgs.msg import CrazyflieDesired, CrazyflieLanding
from std_msgs.msg import Bool

###########################################
# Global Parameters
###########################################
loop_rate = 100.0 # Hz

# selet trajectory to follow
# traj_name = 'hover_80cm_30s'    # starts at x=0.25, y=-0.25
traj_name = 'hover_1m_300s'     # starts at x = 0.0, y=0.0

###########################################
# Functions
###########################################
def read_trajectory_from_file(traj_name):
    """
    Reads the trajectory data from a file and returns numpy arrays for time, x, y, z, and yaw.
    
    Arguments:
    file_name (str): The name of the file to read the trajectory from.
    
    Returns:
    time, x_pos, y_pos, z_pos, yaw (numpy arrays): Arrays of time, x, y, z, yaw data.
    """
    # Load the trajectory data from the file, skip the header row
    traj_location = '/home/parallels/JMA_ws/src/trajectories/saved_trajectories/' + traj_name
    data = np.loadtxt(traj_location, delimiter=' ', skiprows=1)
    
    # # Extract each column from the loaded data
    # time = data[:, 0]
    # x_pos = data[:, 1]
    # y_pos = data[:, 2]
    # z_pos = data[:, 3]
    # yaw = data[:, 4]
    
    # return time, x_pos, y_pos, z_pos, yaw
    return data

###########################################
# Node Class
###########################################
class PositionControlNode(Node):
    def __init__(self):
        super().__init__("crazyflie_positioning")

        # declare ID as a parameter, set default to 1
        self.declare_parameter("id",1)
        # get the parameter value from the launch script, or use the default
        self.id = self.get_parameter("id").value
        self.formatted_id = f"{self.id:02d}"

        self.get_logger().info("cf" + self.formatted_id + " Positioning Launched")

        # set the URI for the drone, as well as subscriber and publisher topics
        self.uri = 'radio://0/80/2M/E7E7E7E7' + self.formatted_id
        sub_topic = '/cf' + self.formatted_id + '/pose'
        pub_topic = '/cf' + self.formatted_id + '/desired'
        test_complete_topic = '/cf' + self.formatted_id + '/test_complete'

        # set initial landing and take of flags and values
        self.first_position = True
        self.first_landing_command = True
        self.take_off = True
        self.landing = False
        self.start_test = False
        self.z_take_off = 0.0
        self.time_shift = 0.0

        self.init_flag = True
        self.xy_start_flag = True

        # trajectory loading parameters
        self.k = 0 # iteration variable for trajectory indexing
        global traj_name
        self.traj_data = read_trajectory_from_file(traj_name)   # columns are (time, x, y, z, yaw)

        # create subscribers and publishers for the crazyflie
        # this one subscribes to MoCap data
        self.position_sub = self.create_subscription(PoseStamped,sub_topic,self.position_callback,10)
        # this one publishes desired (x,y,z) position to the crazyflie_control.py node
        self.desired_position_pub = self.create_publisher(CrazyflieDesired,pub_topic,10)
        # this subscribes to the landing topic to determine if landing is necessary
        self.landing_sub = self.create_subscription(CrazyflieLanding,'land',self.landing_callback,10)
        self.start_test_sub = self.create_subscription(Bool,'start_test',self.start_test_callback,10)

        self.test_comlete_pub = self.create_publisher(Bool,test_complete_topic,10)
        self.test_complete_msg = Bool()
        self.test_complete_msg.data = False
        

        # get the initialization time
        self.init_time = time.time()
        self.t = time.time() - self.init_time

    def euler_from_quaternion(self):
        """takes the quaternions from MoCap and converts them to Euler.
        Way easier to visualize and verify"""
        t0 = +2.0 * (self.qw * self.qx + self.qy * self.qx)
        t1 = +1.0 - 2.0 * (self.qx * self.qx + self.qy * self.qy)

        t2 = +2.0 * (self.qw * self.qy - self.qz * self.qx)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2

        t3 = +2.0 * (self.qw * self.qz + self.qx * self.qy)
        t4 = +1.0 - 2.0 * (self.qy * self.qy + self.qz * self.qz)
        self.yaw = math.atan2(t3, t4) * 180.0 / math.pi
        

    def position_callback(self,msg):
        if self.first_position:
            self.x0 = msg.pose.position.x
            self.y0 = msg.pose.position.y
            self.z0 = msg.pose.position.z
            self.z_take_off = self.z0

            self.qx = msg.pose.orientation.x
            self.qy = msg.pose.orientation.y
            self.qz = msg.pose.orientation.z
            self.qw = msg.pose.orientation.w
            # convert quaternions to euler angles
            self.euler_from_quaternion()
            self.yaw0 = self.yaw

            self.first_position = False
            self.create_timer((1.0/loop_rate),self.desired_position_publisher)
        elif self.first_landing_command and self.landing:
            self.get_logger().info('Landing...')
            self.x_land = msg.pose.position.x
            self.y_land = msg.pose.position.y
            self.z_land = msg.pose.position.z
            self.first_landing_command = False
        self.z = msg.pose.position.z

    def desired_position_publisher(self):
        msg = CrazyflieDesired()
        # uncomment sections below for different behaviors
        self.t = time.time() - self.init_time

        hover_height = self.traj_data[0,3]
        
        if self.init_flag:
            msg.x_des = self.x0
            msg.y_des = self.y0
            msg.z_des = self.z0
            msg.yaw_des = self.yaw0

            if self.t > 5.0:
                self.init_flag = False
                self.init_time = time.time()
        else:

            # set up take off procedure
            # the drone will slowly rise to a desired height
            # once at the desired height, the desired trajectory will begin

            if self.take_off and self.z_take_off < hover_height:
                self.z_take_off = self.sigmoid_traj(self.t, self.z0, 1.001*hover_height)
                
                msg.z_des = self.z_take_off

                if self.z_take_off < 0.25:
                    msg.x_des = self.x0
                    msg.y_des = self.y0
                    msg.yaw_des = self.yaw0
                else:
                    if self.xy_start_flag:
                        self.time_shift = time.time() - self.init_time
                        self.xy_start_flag = False

                    if self.traj_data[0,1] == 0.0:
                        if self.x0 > 0.0:
                            msg.x_des = self.sigmoid_traj(self.t - self.time_shift, self.x0, 0.001)
                        else:
                            msg.x_des = self.sigmoid_traj(self.t - self.time_shift, self.x0, -0.001)
                    else:
                        msg.x_des = self.sigmoid_traj(self.t - self.time_shift, self.x0, 1.001*self.traj_data[0,1])

                    if self.traj_data[0,2] == 0.0:
                        if self.y0 > 0.0:
                            msg.y_des = self.sigmoid_traj(self.t - self.time_shift, self.y0, 0.001)
                        else:
                            msg.y_des = self.sigmoid_traj(self.t - self.time_shift, self.y0, -0.001)
                    else:
                        msg.y_des = self.sigmoid_traj(self.t - self.time_shift, self.y0, 1.001*self.traj_data[0,2])

                    
                    msg.yaw_des = self.traj_data[0,4] # yaw from trajectory file

                if self.z_take_off > hover_height and abs(msg.x_des) > abs(self.traj_data[0,1]) and abs(msg.y_des) > abs(self.traj_data[0,2]):
                    self.take_off = False
                    self.get_logger().info("cf" + self.formatted_id + " hover position reached")
                    self.init_time = time.time()
                    self.t = time.time() - self.init_time
            elif self.start_test and not self.test_complete_msg.data:
                ###############################################
                # Follow trajectory
                ###############################################

                # Hover
                msg.x_des = self.traj_data[self.k,1] # x from trajectory file
                msg.y_des = self.traj_data[self.k,2] # y from trajectory file
                msg.z_des = self.traj_data[self.k,3] # z from trajectory file
                msg.yaw_des = self.traj_data[0,4] # yaw from trajectory file

                if self.k < len(self.traj_data)-1:
                    self.k += 1
                else:
                    self.test_complete_msg.data = True
                    self.get_logger().info("cf" + self.formatted_id + ": test completed, holding position")
            else:

                if self.k == 0:
                    msg.x_des = self.traj_data[0,1] # x from trajectory file
                    msg.y_des = self.traj_data[0,2] # y from trajectory file
                    msg.z_des = self.z_take_off
                    msg.yaw_des = self.traj_data[0,4] # yaw from trajectory file
                else:
                    msg.x_des = self.traj_data[self.k,1] # hold last x position
                    msg.y_des = self.traj_data[self.k,2] # hold last y position
                    msg.z_des = self.traj_data[self.k,3] # hold last z position
                    msg.yaw_des = self.traj_data[0,4] # yaw from trajectory file

        # set up landing producre
        # the drone will slowly descend to a 5 cm
        # another script will then kill the motors and land the drone
        if self.landing and not self.first_landing_command:
            # need to decide a better landing procedure
            if self.z_land > 0.075:
                self.z_land -= 0.001
            msg.x_des = self.x_land
            msg.y_des = self.y_land
            msg.z_des = self.z_land

            

        self.desired_position_pub.publish(msg)
        self.test_comlete_pub.publish(self.test_complete_msg)

        if self.start_test and self.k % 1000 == 0:
            self.get_logger().info("Time: " + str(self.k/100) + "s ||| cf" + self.formatted_id + " moving to x: " + str(msg.x_des) + " y: " + str(msg.y_des) + " z: " + str(msg.z_des))


    def landing_callback(self, msg):
        self.landing = msg.land

    def start_test_callback(self, msg):
        self.start_test = msg.data

    def sigmoid_traj(self, t, start_height, max_height):
        # alpha specifies the ratio of the max height to compare to for the midpoint calculation
        # larger alpha will take longer but limit to max speed better
        max_height = max_height - start_height
        if max_height < 0.0:
            sign_val = -1.0
            max_height = abs(max_height)
        else:
            sign_val = 1.0

        set_vel = 0.07 # m/s

        # t0 = -(1/(2*max_slope)) * math.log((max_height - alpha*max_height) / (alpha*max_height))
        t0 = max_height/ (set_vel*2)
        slope = (4*set_vel*3) / max_height
        return  start_height + (sign_val *(max_height / (1 + math.exp(-slope * (t - t0)))))

def main(args=None):
    """Main function. Will run automatically on starting the node."""
    try:
        rclpy.init(args=args)
        node = PositionControlNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()