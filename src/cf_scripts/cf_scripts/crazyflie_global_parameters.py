"""Created and adapted by Jakon Allred
May 2024
For use in DARC Lab 
For storing global parameters
to be used across all nodes"""
#!/usr/bin/env python 3

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rcl_interfaces.msg import SetParametersResult
from custom_msgs.msg import CrazyflieLanding
import rclpy.subscription

class GlobalParametersNode(Node):

    def parameter_callback(self, params):
        # a callback for processing global parameter changes
        for param in params:
            if param.name == 'land' and param.type_ == Parameter.Type.BOOL:
                self.land_param = param.value
                self.get_logger().info(f'Landing command set to: {param.value}')
        return SetParametersResult(successful=True)

    def __init__(self):
        super().__init__("crazyflie_global_parameters")

        self.get_logger().info('Crazyflie Global Parameters launched')

        # declare the landing parameter, default value of false
        self.declare_parameter('land',False)
        # read the landing parameter and store it
        self.land_param = self.get_parameter('land').value
        # add a callback for when the parameter changes
        self.add_on_set_parameters_callback(self.parameter_callback)

        # create a publisher for all other nodes to subscribe to
        # the publisher sends False until the parameter is set to true
        # once the parameter is set to True all drones will land and
        # the data will be printed
        self.landing_publisher = self.create_publisher(CrazyflieLanding,'/land',10)
        self.create_timer(1.0,self.publish_landing)

    def publish_landing(self):
        # publishes the land parameter
        msg = CrazyflieLanding()
        msg.land = self.land_param
        self.landing_publisher.publish(msg)

def main(args=None):
    """Main function. Will run automatically on starting the node."""
    try:
        rclpy.init(args=args)
        node = GlobalParametersNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()