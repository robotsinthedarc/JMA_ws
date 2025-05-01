from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    global_param_node = Node(
        package="crazyflie_test_pkg",
        executable="crazyflie_global_parameters"
    )
    ld.add_action(global_param_node)

    # adjust the number of drones to match the swarm
    # drones should be added in order of URI
    # ie for 1 drone, uri 10 should be used
    # for 2 drones, uris 10 and 11 should be used
    # for 3 drones, 10, 11, and 12 etc.
    num_drones = 1
    ids = 0

    if num_drones >= 1:
        ids += 1
        fly10_control_node = Node(
            package="crazyflie_test_pkg",
            executable="crazyflie_control",
            name="fly10_control", # this renames the node, preventing issues
            parameters=[
                {"id":10}
            ]
        )
        fly10_data_node = Node(
            package="crazyflie_test_pkg",
            executable="crazyflie_data_reader",
            name="fly10_data_reader",
            parameters=[
                {"id":10}
            ]
        )
        fly10_desired_node = Node(
            package="crazyflie_test_pkg",
            executable="crazyflie_positioning",
            name="fly10_positioning",
            parameters=[
                {"id":10}
            ]
        )
        ld.add_action(fly10_control_node)
        ld.add_action(fly10_data_node)
        ld.add_action(fly10_desired_node)


    if num_drones >= 2:
        ids += 1
        fly11_control_node = Node(
            package="crazyflie_test_pkg",
            executable="crazyflie_control",
            name="fly11_control",
            parameters=[
                {"id":11}
            ]
        )
        fly11_data_node = Node(
            package="crazyflie_test_pkg",
            executable="crazyflie_data_reader",
            name="fly11_data_reader",
            parameters=[
                {"id":11}
            ]
        )
        fly11_desired_node = Node(
            package="crazyflie_test_pkg",
            executable="crazyflie_positioning",
            name="fly11_positioning",
            parameters=[
                {"id":11}
            ]
        )
        ld.add_action(fly11_control_node)
        ld.add_action(fly11_data_node)
        ld.add_action(fly11_desired_node)

    swarm_node = Node(
        package="crazyflie_test_pkg",
        executable="swarm_control",
        parameters=[
            {"ids":ids}
        ]
    )

    ld.add_action(swarm_node)

    return ld
