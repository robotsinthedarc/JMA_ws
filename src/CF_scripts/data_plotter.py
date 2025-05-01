
import matplotlib.pyplot as plt
import numpy as np
import math

timestamp = '2024_07_11_11_38'
num_drones = 2
plot_error = False
plot_angles = False

file_name1 = '/home/darc/cf_ws/cf_10_data_' + timestamp
content1 = np.loadtxt(file_name1)

if num_drones > 1:
    file_name2 = '/home/darc/cf_ws/cf_11_data_' + timestamp
    content2 = np.loadtxt(file_name2)

time1 = (content1[:,0] - content1[0,0]) / 1000.0 
acc_x1 = content1[:,1]
acc_y1 = content1[:,2]
acc_z1 = content1[:,3]
gyro_x1 = content1[:,4]
gyro_y1 = content1[:,5]
gyro_z1 = content1[:,6]
x1 = content1[:,7]
y1 = content1[:,8]
z1 = content1[:,9]
x_des1 = content1[:,10]
y_des1 = content1[:,11]
z_des1 = content1[:,12]
roll_imu1 = content1[:,13]
pitch_imu1 = content1[:,14]
yaw_imu1 = content1[:,15]
roll_mocap1 = content1[:,16]
pitch_mocap1 = content1[:,17]
yaw_mocap1 = content1[:,18]
roll_des1 = content1[:,19]
pitch_des1 = content1[:,20]
yaw_rate_des1 = content1[:,21]
m11 = content1[:,22]
m21 = content1[:,23]
m31 = content1[:,24]
m41 = content1[:,25]
voltage1 = content1[:,26]
current1 = content1[:,27]
x_err1 = content1[:,28]
x_err_dot1 = content1[:,29]
x_err_int1 = content1[:,30]
y_err1 = content1[:,31]
y_err_dot1 = content1[:,32]
y_err_int1 = content1[:,33]
z_err1 = content1[:,34]
z_err_dot1 = content1[:,35]
z_err_int1 = content1[:,36]

if num_drones > 1:
    time2 = (content2[:,0] - content2[0,0]) / 1000.0
    acc_x2 = content2[:,1]
    acc_y2 = content2[:,2]
    acc_z2 = content2[:,3]
    gyro_x2 = content2[:,4]
    gyro_y2 = content2[:,5]
    gyro_z2 = content2[:,6]
    x2 = content2[:,7]
    y2 = content2[:,8]
    z2 = content2[:,9]
    x_des2 = content2[:,10]
    y_des2 = content2[:,11]
    z_des2 = content2[:,12]
    roll_imu2 = content2[:,13]
    pitch_imu2 = content2[:,14]
    yaw_imu2 = content2[:,15]
    roll_mocap2 = content2[:,16]
    pitch_mocap2 = content2[:,17]
    yaw_mocap2 = content2[:,18]
    roll_des2 = content2[:,19]
    pitch_des2 = content2[:,20]
    yaw_rate_des2 = content2[:,21]
    m12 = content2[:,22]
    m22 = content2[:,23]
    m32 = content2[:,24]
    m42 = content2[:,25]
    voltage2 = content2[:,26]
    current2 = content2[:,27]
    x_err2 = content2[:,28]
    x_err_dot2 = content2[:,29]
    x_err_int2 = content2[:,30]
    y_err2 = content2[:,31]
    y_err_dot2 = content2[:,32]
    y_err_int2 = content2[:,33]
    z_err2 = content2[:,34]
    z_err_dot2 = content2[:,35]
    z_err_int2 = content2[:,36]

plt.figure(1)
plt.plot(x1,y1,'-',label='Actual 10',markersize=1)
#plt.plot(x_des1,y_des1,'o-',label='Desired 10',markersize=1)
plt.plot(x2,y2,'-',label='Actual 11',markersize=1)
plt.xlabel('x [m]')
plt.ylabel('y [m]')
plt.title('x and y')
plt.gca().set_aspect('equal')
plt.legend()

plt.figure(2)
plt.plot(time1,z1,'-',label='Actual',markersize=1)
plt.plot(time1,z_des1,'--',label='Desired',markersize=1)
plt.xlabel('Time [s]')
plt.ylabel('z [m]')
plt.title('Fly 10')
plt.legend()

plt.figure(3)
plt.plot(time1,y1,'-',label='Actual')
plt.plot(time1,y_des1,'--',label='Desired')
plt.xlabel('Time [s]')
plt.ylabel('y [m]')
plt.title('Fly10 y vs time')
plt.legend()

plt.figure(4)
plt.plot(time1,x1,'-',label='Actual')
plt.plot(time1,x_des1,'--',label='Desired')
plt.xlabel('Time [s]')
plt.ylabel('x [m]')
plt.title('Fly10 x vs time')
plt.legend()

if num_drones > 1:
    plt.figure(5)
    plt.plot(x2,y2,'-',label='Actual 11',markersize=1)
    plt.plot(x_des2,y_des2,'o-',label='Desired 11',markersize=1)
    plt.xlabel('x [m]')
    plt.ylabel('y [m]')
    plt.title('x and y')
    plt.gca().set_aspect('equal')
    plt.legend()

    plt.figure(6)
    plt.plot(time2,z2,'-',label='Actual',markersize=1)
    plt.plot(time2,z_des2,'--',label='Desired',markersize=1)
    plt.xlabel('Time [s]')
    plt.ylabel('z [m]')
    plt.title('Fly 11')
    plt.legend()

    plt.figure(7)
    plt.plot(time2,y2,'-',label='Actual')
    plt.plot(time2,y_des2,'--',label='Desired')
    plt.xlabel('Time [s]')
    plt.ylabel('y [m]')
    plt.title('Fly11 y vs time')
    plt.legend()

    plt.figure(8)
    plt.plot(time2,x2,'-',label='Actual')
    plt.plot(time2,x_des2,'--',label='Desired')
    plt.xlabel('Time [s]')
    plt.ylabel('x [m]')
    plt.title('Fly11 x vs time')
    plt.legend()

if plot_error:
    plt.figure(9)
    plt.plot(time1,x_err1,'-',label='e')
    #plt.plot(time1,x_err_dot1,'-',label='e_dot')
    #plt.plot(time1,x_err_int1,'-',label='e_int')
    plt.xlabel('Time [s]')
    plt.ylabel('Error')
    plt.title('Fly 10 x-Error vs time')
    plt.legend()

    plt.figure(10)
    plt.plot(time1,y_err1,'-',label='e')
    #plt.plot(time1,y_err_dot1,'-',label='e_dot')
    #plt.plot(time1,y_err_int1,'-',label='e_int')
    plt.xlabel('Time [s]')
    plt.ylabel('Error')
    plt.title('Fly 10 y-Error vs time')
    plt.legend()

    if num_drones > 1:
        plt.figure(11)
        plt.plot(time2,x_err2,'-',label='e')
        #plt.plot(time2,x_err_dot2,'-',label='e_dot')
        #plt.plot(time2,x_err_int2,'-',label='e_int')
        plt.xlabel('Time [s]')
        plt.ylabel('Error')
        plt.title('Fly 11 x-Error vs time')
        plt.legend()

        plt.figure(12)
        plt.plot(time2,y_err2,'-',label='e')
        #plt.plot(time2,y_err_dot2,'-',label='e_dot')
        #plt.plot(time2,y_err_int2,'-',label='e_int')
        plt.xlabel('Time [s]')
        plt.ylabel('Error')
        plt.title('Fly 11 y-Error vs time')
        plt.legend()

if plot_angles:
    plt.figure(13)
    plt.plot(time1,yaw_imu1,'-',label='IMU')
    plt.plot(time1,yaw_mocap1,'-',label='MoCap')
    plt.xlabel('Time [s]')
    plt.ylabel('Yaw [deg]')
    plt.title('Fly 10 Yaw')
    plt.legend()

    plt.figure(14)
    plt.plot(time1,roll_des1,'-',label='Desired')
    plt.plot(time1,roll_imu1,'-',label='IMU')
    plt.xlabel('Time [s]')
    plt.ylabel('Roll [deg]')
    plt.title('Fly 10 Roll vs Time')
    plt.legend()

    if num_drones > 1:
        plt.figure(15)
        plt.plot(time2,yaw_imu2,'-',label='IMU')
        plt.plot(time2,yaw_mocap2,'-',label='MoCap')
        plt.xlabel('Time [s]')
        plt.ylabel('Yaw [deg]')
        plt.title('Fly 11 Yaw')
        plt.legend()

        plt.figure(16)
        plt.plot(time2,roll_des2,'-',label='Desired')
        plt.plot(time2,roll_imu2,'-',label='IMU')
        plt.xlabel('Time [s]')
        plt.ylabel('Roll [deg]')
        plt.title('Fly 11 Roll vs Time')
        plt.legend()

plt.show()