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
from std_msgs.msg import Bool
import rclpy.subscription

class GlobalParametersNode(Node):

    def parameter_callback(self, params):
        # a callback for processing global parameter changes
        for param in params:
            # check for landing parameter
            if param.name == 'land' and param.type_ == Parameter.Type.BOOL:
                self.land_param = param.value
                self.get_logger().info(f'Landing command set to: {param.value}')

            # Check for the 'start_test' parameter update
            if param.name == 'start_test' and param.type_ == Parameter.Type.BOOL:
                self.start_test_param = param.value
                self.get_logger().info(f'Start test command set to: {param.value}')

        return SetParametersResult(successful=True)

    def __init__(self):
        super().__init__("crazyflie_global_parameters")

        self.get_logger().info('Crazyflie Global Parameters launched')

        # declare the landing parameter, default value of false
        self.declare_parameter('land',False)
        # Declare the start_test parameter, default value of False
        self.declare_parameter('start_test', False)

        # read the parameters and store them
        self.land_param = self.get_parameter('land').value
        self.start_test_param = self.get_parameter('start_test').value

        # add a callback for when the parameter changes
        self.add_on_set_parameters_callback(self.parameter_callback)

        # create a publisher for all other nodes to subscribe to
        # the publisher sends False until the parameter is set to true
        # once the parameter is set to True all drones will land and
        # the data will be printed
        self.landing_publisher = self.create_publisher(CrazyflieLanding,'/land',10)
        self.create_timer(0.01,self.publish_landing) # 100 Hz

        self.start_test_publisher = self.create_publisher(Bool, '/start_test', 10)
        self.create_timer(0.25, self.publish_start_test) # 4 Hz

    def publish_landing(self):
        # publishes the land parameter
        msg = CrazyflieLanding()
        msg.land = self.land_param
        self.landing_publisher.publish(msg)

    def publish_start_test(self):
        # Publishes the start_test parameter as a boolean
        msg = Bool()
        msg.data = self.start_test_param
        self.start_test_publisher.publish(msg)

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