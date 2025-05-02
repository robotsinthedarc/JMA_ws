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
from custom_msgs import cf_desPos, cf_landing

class PositionControlNode(Node):
    def __init__(self):
        super().__init__("crazyflie_positioning")

        # declare ID as a parameter, set default to 10
        self.declare_parameter("id",1)
        # get the parameter value from the launch script, or use the default
        self.id = self.get_parameter("id").value
        self.formatted_id = f"{self.id:02d}"

        self.get_logger().info("cf " + self.formatted_id + " Positioning Launched")

        # set the URI for the drone, as well as subscriber and publisher topics
        self.uri = 'radio://0/80/2M/E7E7E7E7' + self.formatted_id
        sub_topic = '/cf' + self.formatted_id + '/pose'
        pub_topic = '/cf' + self.formatted_id + '/desired'

        # set initial landing and take of flags and values
        self.first_position = True
        self.first_landing_command = True
        self.take_off = False #True
        self.landing = False
        self.z_take_off = 0.0

        # create subscribers and publishers for the crazyflie
        # this one subscribes to MoCap data
        self.position_sub = self.create_subscription(PoseStamped,sub_topic,self.position_callback,10)
        # this one publishes desired (x,y,z) position to the crazyflie_control.py node
        self.desired_position_pub = self.create_publisher(CrazyflieDesired,pub_topic,10)
        # this subscribes to the landing topic to determine if landing is necessary
        self.landing_sub = self.create_subscription(CrazyflieLanding,'land',self.landing_callback,10)
        time.sleep(2.0)

        # get the initialization time
        self.init_time = time.time()
        self.t = time.time() - self.init_time

        # the period of a revolution if the drone is flying in a circle
        self.omega = 1.0

        

    def position_callback(self,msg):
        if self.first_position:
            self.x0 = msg.pose.position.x
            self.y0 = msg.pose.position.y
            self.z0 = msg.pose.position.z
            self.first_position = False
            self.create_timer(0.01,self.desired_position_publisher)
        if self.first_landing_command and self.landing:
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
        msg.z_des = 0.5

        # set up take off procedure
        # the drone will slowly rise to a desired height
        # once at the desired height, the desired trajectory will begin
        if self.take_off and self.z_take_off < 0.3:
            self.z_take_off += 0.001
            msg.z_des = self.z_take_off
            msg.x_des = self.x0
            msg.y_des = self.y0
            if self.z_take_off > 0.3:
                self.take_off = False
                self.init_time = time.time()
                self.t = time.time() - self.init_time

        # set up landing producre
        # the drone will slowly descend to a 5 cm
        # another script will then kill the motors and land the drone
        if self.landing and not self.first_landing_command:
            # need to decide a better landing procedure
            if self.z_land > 0.05:
                self.z_land -= 0.001
            msg.x_des = self.x_land
            msg.y_des = self.y_land
            msg.z_des = self.z_land

        self.desired_position_pub.publish(msg)

    def landing_callback(self, msg):
        self.landing = msg.land

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