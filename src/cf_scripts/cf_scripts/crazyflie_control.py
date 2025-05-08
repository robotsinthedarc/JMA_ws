"""Created and adapted by Jakon Allred
May 2024
For use in DARC Lab on CrazyFlie Bolt
This will be used to control any general Crazyflie
Controller gains will be adjusted according to 
the URI being used"""
#!/usr/bin/env python 3

import time
import math

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped
from custom_msgs.msg import CrazyflieCommands, CrazyflieDesired

###########################################
# Global Parameters
###########################################
loop_rate = 100.0 # Hz

###########################################
# Node Class
###########################################
class FlyControlNode(Node):
    def __init__(self):
        super().__init__("crazyflie_control")

        # declare drone ID as a parameter, set default to 1
        self.declare_parameter("id",1)
        # get the parameter from the launch script, or use the default
        self.id = self.get_parameter("id").value
        self.formatted_id = f"{self.id:02d}"

        self.get_logger().info("cf " + str(self.formatted_id) + " Control Launched")

        # set the URI for the drone, and name all of the subscriber and publisher topics
        self.uri = 'radio://0/80/2M/E7E7E7E7' + self.formatted_id
        sub_topic1 = '/cf' + self.formatted_id + '/pose'
        sub_topic2 = '/cf' + self.formatted_id + '/desired'
        pub_topic = '/cf' + self.formatted_id + '/commands'

        # create subscribers and publishers
        # this subscriber gets the pose from MoCap
        self.position_sub = self.create_subscription(PoseStamped,sub_topic1,self.position_callback,10)
        # this subscriber gets the desired position from the crazyflie_positioning.py node
        self.desired_position_sub = self.create_subscription(CrazyflieDesired,sub_topic2,self.desired_position_callback,10)
        # this publisher publishes commands, which ar sent to the swarm_control.py node
        self.motor_command_publisher = self.create_publisher(CrazyflieCommands,pub_topic,10)
        self.create_timer((1.0/loop_rate),self.publish_commands)

        # set an initial flag to get the first position of the drone
        self.first_position = True

        self.z_desired = 0.0 # desired height in meters
        self.roll_des = 0.0 # initial desired roll in degrees
        self.pitch_des = 0.0 # initial desired pitch in degrees
        self.yaw_des = 0.0 # initial desired yaw in degrees
        self.yaw_rate_des = 0.0 # initial desired yaw rate in degrees/sec

        self.z_err = 0.0 # initial z error
        self.z_err_prev = 0.0 # initial previous z error
        self.z_err_dot = 0.0 # initial z error derivative
        self.z_err_int = 0.0 # initial integral of the z error
        self.xy_int_max = 1.0 # maximum xy integral error (to prevent windup)
        self.xy_int_min = -self.xy_int_max

        self.z_int_max = 2.0 # maximum zintegral error (to prevent windup)
        self.z_int_min = -self.z_int_max

        self.thrust = 0 # initial thrust value to be sent to the drone

        self.y_err = 0.0 # initial y error
        self.y_err_prev = 0.0 # initial previous y error
        self.y_err_dot = 0.0 # initial y error derivative
        self.y_err_int = 0.0 # initial integral of the y error

        self.x_err = 0.0 # initial x error
        self.x_err_prev = 0.0 # initial previous x error
        self.x_err_dot = 0.0 # initial x error derivative
        self.x_err_int = 0.0 # initial integral of the x error

        self.yaw_err = 0.0 # initial yaw error
        self.yaw_err_prev = 0.0 # initial previous yaw error
        self.yaw_err_dot = 0.0 # initial yaw error derivative

        # set controller gains
        # the gains can be set by ID, depending on the parameter set earlier
        
        # gains tested 06/24/24. Need continued testing
        # gains for the z dierection appear fine for both drones
        # tuning needs continued in x and y directions

        # self.kp_z = 115.0 # Jakon's gains
        # self.kd_z = 40.0
        # self.ki_z = 25.0
        self.kp_z = 115.0 # Jacob's gains
        self.kd_z = 38.0
        self.ki_z = 23.0

        # tuned for x-step
        # self.kp_xy = 60.0
        # self.kd_xy = 50.0
        # self.ki_xy = 0.0

        # tuned on y-step (05-07-25)
        self.kp_xy = 78.0
        self.kd_xy = 61.0
        self.ki_xy = 7.0

        self.kp_yaw = 0.5
        self.kd_yaw = 0.0

        
            
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

        # convert quaternions to euler angles
        self.euler_from_quaternion()

        # read and set initial (x,y,z) and yaw values
        if self.first_position:
            self.first_position = False
            self.x0 = msg.pose.position.x
            self.y0 = msg.pose.position.y
            self.z0 = msg.pose.position.z
            self.yaw0 = self.yaw
            self.x_desired = self.x0
            self.y_desired = self.y0
            self.yaw_desired = self.yaw0

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

    def motor_controller(self):
        """Function for calculating necessary motor thrust
        Desired pitch and roll are calculated too
        Due to drift of yaw, a yaw controller should be added as well"""
        # calculate elapsed time since last time through the loop
        self.t_prev = self.t
        self.t = time.time() - self.init_time
        delta_t = self.t - self.t_prev

        # calculate z error with derivative and integral
        self.z_err_prev = self.z_err
        self.z_err = self.z_desired - self.z
        self.z_err_dot = (self.z_err - self.z_err_prev) / delta_t
        self.z_err_int += self.z_err * delta_t
        # if the integral error is too high, cap it
        if self.z_err_int > self.z_int_max:
            self.z_err_int = self.z_int_max
        if self.z_err_int < self.z_int_min:
            self.z_err_int = self.z_int_min

        # calculate y error with derivative and integral
        # the y error is negative because the crazyflie coordinate system
        # is flipped from the global coordinate system, but only for the y-direction
        self.y_err_prev = self.y_err
        self.y_err =  -(self.y_desired - self.y)
        self.y_err_dot = (self.y_err - self.y_err_prev) / delta_t
        self.y_err_int += self.y_err * delta_t
        # if the integral error is too high, cap it
        if self.y_err_int > self.xy_int_max:
            self.y_err_int = self.xy_int_max
        if self.y_err_int < self.xy_int_min:
            self.y_err_int = self.xy_int_min

        # calculate x error with derivative and integral
        self.x_err_prev = self.x_err
        self.x_err = self.x_desired - self.x
        self.x_err_dot = (self.x_err - self.x_err_prev) / delta_t
        self.x_err_int += self.x_err * delta_t
        # if the integral error is too high, cap it
        if self.x_err_int > self.xy_int_max:
            self.x_err_int = self.xy_int_max
        if self.x_err_int < self.xy_int_min:
            self.x_err_int = self.xy_int_min

        self.yaw_err_prev = self.yaw_err
        self.yaw_err = ((self.yaw - self.yaw_desired) + 180.0) % 360.0 - 180.0 # normalized between -180 and 180 (needed to flip the error calculation, due to rotations??)
        self.yaw_err_dot = (self.yaw_err - self.yaw_err_prev) / delta_t

        # calculate thrust
        # map force value to thrust value
        # thrust ranges from 20000 to 60000
        force = self.kp_z * self.z_err + self.kd_z * self.z_err_dot + self.ki_z * self.z_err_int
        self.thrust = math.floor(25000 + 500*force)
        if self.thrust > 60000: # higher than max, cap the thrust
            self.thrust = 60000
        elif self.thrust < 20000: # lower than min, set to min
            self.thrust = 20000

        # calculate desired roll and pitch in world coordinates
        roll_des = self.kp_xy * self.y_err + self.kd_xy * self.y_err_dot + self.ki_xy * self.y_err_int
        pitch_des = self.kp_xy * self.x_err + self.kd_xy * self.x_err_dot + self.ki_xy * self.x_err_int
        self.yaw_rate_des = self.kp_yaw * self.yaw_err + self.kd_yaw * self.yaw_err_dot

        # transform desired roll and pitch to body coordinates
        # self.roll_des = roll_des * math.cos(-self.yaw * math.pi / 180.0) - pitch_des * math.sin(-self.yaw * math.pi / 180.0)
        # self.pitch_des = roll_des * math.sin(-self.yaw * math.pi / 180.0) + pitch_des * math.cos(-self.yaw * math.pi / 180.0)
        self.roll_des = roll_des * math.cos(-self.yaw * math.pi / 180.0) - pitch_des * math.sin(-self.yaw * math.pi / 180.0)
        self.pitch_des = (roll_des * math.sin(-self.yaw * math.pi / 180.0) + pitch_des * math.cos(-self.yaw * math.pi / 180.0))
        if self.roll_des > 45.0:
            self.roll_des = 45.0
        elif self.roll_des < -45.0:
            self.roll_des = -45.0
        if self.pitch_des > 45.0:
            self.pitch_des = 45.0
        elif self.pitch_des < -45.0:
            self.pitch_des = -45.0

    def publish_commands(self):
        """Publishes vehicle URI and
        desired roll, pitch, and yaw rate"""
        msg = CrazyflieCommands()
        if not self.first_position:
            self.motor_controller()
            msg.x = self.x
            msg.y = self.y
            msg.z = self.z
            msg.x_desired = self.x_desired
            msg.y_desired = self.y_desired
            msg.z_desired = self.z_desired
            msg.yaw_desired = self.yaw_desired
            msg.roll_mocap = self.roll
            msg.pitch_mocap = self.pitch
            msg.yaw_mocap = self.yaw
            msg.x_err = self.x_err
            msg.x_err_dot = self.x_err_dot
            msg.x_err_int = self.x_err_int
            msg.y_err = self.y_err
            msg.y_err_dot = self.y_err_dot
            msg.y_err_int = self.y_err_int
        msg.uri = self.uri
        msg.desired_roll = self.roll_des
        msg.desired_pitch = self.pitch_des
        msg.desired_yaw_rate = self.yaw_rate_des # once the yaw controller is implemented, this will need to change
        msg.thrust = self.thrust

        self.motor_command_publisher.publish(msg)

    def desired_position_callback(self,msg):
        # reads the desired position from the crazyflie_positioning.py node
        # desired yaw may be added later
        self.x_desired = msg.x_des
        self.y_desired = msg.y_des
        self.z_desired = msg.z_des
        self.yaw_desired = msg.yaw_des

def main(args=None):
    """Main function. Will run automatically on starting the node."""
    try:
        rclpy.init(args=args)
        node = FlyControlNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
