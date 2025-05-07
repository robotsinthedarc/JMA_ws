"""Created and adapted by Jakon Allred
May 2024
For use in DARC Lab 
For publishing desired positions
to individual Crazyflies"""
#!/usr/bin/env python 3

import math
import time

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped
from custom_msgs.msg import CrazyflieDesired, CrazyflieLanding
from std_msgs.msg import Bool

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

        # set initial landing and take of flags and values
        self.first_position = True
        self.first_landing_command = True
        self.take_off = True
        self.landing = False
        self.start_test = False
        self.z_take_off = 0.0

        self.init_flag = True

        # create subscribers and publishers for the crazyflie
        # this one subscribes to MoCap data
        self.position_sub = self.create_subscription(PoseStamped,sub_topic,self.position_callback,10)
        # this one publishes desired (x,y,z) position to the crazyflie_control.py node
        self.desired_position_pub = self.create_publisher(CrazyflieDesired,pub_topic,10)
        # this subscribes to the landing topic to determine if landing is necessary
        self.landing_sub = self.create_subscription(CrazyflieLanding,'land',self.landing_callback,10)
        self.start_test_sub = self.create_subscription(Bool,'start_test',self.start_test_callback,10)
        

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
            self.create_timer(0.01,self.desired_position_publisher) # 100 Hz
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

            if self.take_off and self.z_take_off < 0.4:
                alpha = 0.6
                self.z_take_off = self.sigmoid_traj(self.t, self.z0, 1.001*0.4, 1.0, alpha)
                msg.z_des = self.z_take_off
                msg.x_des = self.x0
                msg.y_des = self.y0

                if self.z_take_off < 0.25:
                    msg.yaw_des = self.yaw0
                else:
                    msg.yaw_des = 90.0

                if self.z_take_off > 0.4:
                    self.take_off = False
                    self.get_logger().info("cf" + self.formatted_id + " hover position reached")
                    self.init_time = time.time()
                    self.t = time.time() - self.init_time
            elif self.start_test:
                ###############################################
                # Select Trajectory
                ###############################################

                # # circle
                # msg.x_des = self.x0 + 0.5 * math.cos(self.omega * self.t)
                # msg.y_des = self.y0 + 0.5 * math.sin(self.omega * self.t)
                # msg.z_des = 0.5

                # #step
                # if self.t < 5.0:
                #     msg.x_des = self.x0
                #     msg.y_des = self.y0
                #     msg.z_des = 0.5
                # elif self.t >= 5.0 and self.t < 10.0:
                #     msg.x_des = self.x0 + 0.1
                #     msg.y_des = self.y0
                #     msg.z_des = 0.5
                # elif self.t >= 10.0 and self.t < 15.0:
                #     msg.x_des = self.x0 - 0.1
                #     msg.y_des = self.y0
                #     msg.z_des = 0.5
                # elif self.t >= 15.0 and self.t < 20.0:
                #     msg.x_des = self.x0
                #     msg.y_des = self.y0
                #     msg.z_des = 0.5
                # elif self.t >= 20.0 and self.t < 25.0:
                #     msg.x_des = self.x0
                #     msg.y_des = self.y0 + 0.1
                #     msg.z_des = 0.5
                # elif self.t >= 25.0 and self.t < 30.0:
                #     msg.x_des = self.x0
                #     msg.y_des = self.y0 - 0.1
                #     msg.z_des = 0.5
                # elif self.t >= 35.0 and self.t < 40.0:
                #     msg.x_des = self.x0
                #     msg.y_des = self.y0
                #     msg.z_des = 0.5    
                # else:
                #     msg.x_des = self.x0
                #     msg.y_des = self.y0
                #     msg.z_des = 0.5
                
                # Hover
                msg.x_des = self.x0
                msg.y_des = self.y0
                # msg.z_des = 0.5
                msg.z_des = self.z_take_off
                msg.yaw_des = 90.0
            else:
                msg.z_des = self.z_take_off
                msg.x_des = self.x0
                msg.y_des = self.y0
                msg.yaw_des = 90.0

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

    def landing_callback(self, msg):
        self.landing = msg.land

    def start_test_callback(self, msg):
        self.start_test = msg.data

    def sigmoid_traj(self, t, start_height, max_height, max_slope, alpha):
        # alpha specifies the ratio of the max height to compare to for the midpoint calculation
        # larger alpha will take longer but limit to max speed better
        max_height = max_height - start_height

        # t0 = -(1/(2*max_slope)) * math.log((max_height - alpha*max_height) / (alpha*max_height))
        t0 = 5
        return start_height + (max_height / (1 + math.exp(-max_slope * (t - t0))))

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