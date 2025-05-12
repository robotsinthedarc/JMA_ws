#################################################
# IMPORTS
#################################################
import numpy as np
import matplotlib.pyplot as plt
import datetime

#################################################
# FUNCTIONS
#################################################

def create_hover_trajectory(x, y, z, frequency, total_time):
    """
    Function to create a hover trajectory.
    
    Arguments:
    x, y, z (float): The x, y, and z position for the hover.
    frequency (float): Sampling frequency in Hz.
    total_time (float): Total time for the trajectory in seconds.
    
    Returns:
    time (numpy array): Array of time points.
    x_pos, y_pos, z_pos (numpy arrays): Arrays for x, y, z positions.
    yaw (numpy array): Array of yaw angles (constant 90 degrees for hover).
    """
    # Create time vector based on the frequency and total time
    time = np.linspace(0, total_time, int(frequency * total_time))
    
    # Create constant position arrays for x, y, z
    x_pos = np.full_like(time, x)  # Constant x position
    y_pos = np.full_like(time, y)  # Constant y position
    z_pos = np.full_like(time, z)  # Constant z position
    
    # Yaw set to 90 degrees
    yaw = np.full_like(time, 90)
    
    return time, x_pos, y_pos, z_pos, yaw

def save_trajectory_to_file(time, x_pos, y_pos, z_pos, yaw):
    """
    Save the trajectory to a text file where each vector is saved as a column.
    
    Arguments:
    time, x_pos, y_pos, z_pos, yaw (numpy arrays): Data from the trajectory.
    filename (str): Name of the file to save the trajectory.
    """
    # Stack the data into a single array
    trajectory = np.vstack((time, x_pos, y_pos, z_pos, yaw)).T

    file_name = '/home/parallels/JMA_ws/src/trajectories/traj_' + datetime.datetime.now().strftime('%Y_%m_%d_%H_%M')
    
    # Save the trajectory as a text file
    np.savetxt(file_name, trajectory, delimiter=' ', header='Time_(s) X Y Z Yaw_(deg)', comments='')
    print(f"Trajectory saved to {file_name}")


#################################################
# MAIN
#################################################
# Example: Create a hover trajectory
# frequency = 100  # Sample rate in Hz
# total_time = 30  # Total time for trajectory

# Create the hover trajectory
time, x_pos, y_pos, z_pos, yaw = create_hover_trajectory(x=0.0, y=0.0, z=1.0, frequency=100, total_time=300)

# Save the trajectory to a file
save_trajectory_to_file(time, x_pos, y_pos, z_pos, yaw)
