from launch import LaunchDescription
# from launch.actions import DeclareLaunchArgument
# from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    
    ## Used for dynamic inputs but not working, just leave hard coded for now
    # # Use LaunchConfiguration to get the num_drones value
    # num_drones = LaunchConfiguration('num_drones')

    # # Declare an argument for num_drones with a default value of 1
    # ld.add_action(DeclareLaunchArgument('num_drones', default_value='1', description='Number of drones'))
    num_drones = '1'

    global_param_node = Node(
        package="cf_scripts",
        executable="crazyflie_global_parameters"
    )
    ld.add_action(global_param_node)

    # Create nodes for the drones based on the number of drones specified
    # We need to use a substitution to get the num_drones value at runtime
    for i in range(1, int(num_drones) + 1):  # Ensure num_drones gets substituted at runtime
        drone_id = i  

        # Ensure id is formatted to two digits, i.e., "01" to "09"
        formatted_drone_id = f"{drone_id:02d}"

        control_node = Node(
            package="cf_scripts",
            executable="crazyflie_control",
            name=f"cf{formatted_drone_id}_control",  # Dynamic naming with formatted ID
            parameters=[{"id": drone_id}]  # Unique parameter for each drone
        )
        data_node = Node(
            package="cf_scripts",
            executable="crazyflie_data_reader",
            name=f"cf{formatted_drone_id}_data_reader",
            parameters=[{"id": drone_id}]
        )
        positioning_node = Node(
            package="cf_scripts",
            executable="crazyflie_positioning",
            name=f"cf{formatted_drone_id}_positioning",
            parameters=[{"id": drone_id}]
        )

        # Add the nodes to the launch description
        ld.add_action(control_node)
        ld.add_action(data_node)
        ld.add_action(positioning_node)

    # Correct way to use LaunchConfiguration in swarm_node
    swarm_node = Node(
        package="cf_scripts",
        executable="swarm_control",
        parameters=[
            {"ids": int(num_drones)}  # LaunchConfiguration will automatically handle substitution
        ]
    )

    ld.add_action(swarm_node)

    return ld
