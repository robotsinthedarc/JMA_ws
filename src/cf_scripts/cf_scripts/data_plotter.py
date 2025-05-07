
import matplotlib.pyplot as plt
import numpy as np
import math

plt.close('all')

timestamp = '2025_05_06_16_46'
num_drones = 1
plot_error = True
plot_angles = False

file_name1 = '/home/parallels/JMA_ws/src/test_data/cf_01_data_' + timestamp
content1 = np.loadtxt(file_name1)

if num_drones > 1:
    file_name2 = '/home/parallels/JMA_ws/src/test_data/cf_01_data_' + timestamp
    content2 = np.loadtxt(file_name2)

# ([self.timestamp, self.acc_x, self.acc_y, self.acc_z, self.gyro_x, self.gyro_y, self.gyro_z,
#                                      self.x, self.y, self.z, self.x_desired, self.y_desired, self.z_desired,
#                                      self.roll_imu, self.pitch_imu, self.yaw_imu, self.roll_mocap, self.pitch_mocap, self.yaw_mocap,
#                                      self.roll_desired, self.pitch_desired,
#                                      self.m1, self.m2, self.m3, self.m4, self.voltage, self.current,
#                                      self.x_err, self.x_err_dot, self.x_err_int,
#                                      self.y_err, self.y_err_dot, self.y_err_int,
#                                      self.z_err, self.z_err_dot, self.z_err_int])

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
# yaw_rate_des1 = content1[:,21]
m11 = content1[:,21]
m21 = content1[:,22]
m31 = content1[:,23]
m41 = content1[:,24]
voltage1 = content1[:,25]
current1 = content1[:,26]
x_err1 = content1[:,27]
x_err_dot1 = content1[:,28]
x_err_int1 = content1[:,29]
y_err1 = content1[:,30]
y_err_dot1 = content1[:,31]
y_err_int1 = content1[:,32]
z_err1 = content1[:,33]
z_err_dot1 = content1[:,34]
z_err_int1 = content1[:,35]

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
plt.plot(x_des1,y_des1,'o-',label='Desired 10',markersize=1)
# plt.plot(x2,y2,'-',label='Actual 11',markersize=1)
plt.xlabel('x [m]')
plt.ylabel('y [m]')
plt.title('x and y')
plt.gca().set_aspect('equal')
plt.legend()

plt.figure(15)
plt.plot(time1,yaw_mocap1,'-',label='Actual yaw1',markersize=1)
plt.xlabel('x [m]')
plt.ylabel('Yaw [deg]')
# plt.title('x and y')
plt.legend()
plt.title('Yaw Data')

plt.figure(2)
plt.plot(time1,z1,'-',label='Actual',markersize=1)
plt.plot(time1,z_des1,'--',label='Desired',markersize=1)
plt.xlabel('Time [s]')
plt.ylabel('z [m]')
plt.title('Fly 10')
plt.legend()

plt.figure(3, figsize=(10, 12))
plt.subplot(211)
plt.plot(time1,y1,'-',label='Actual')
plt.plot(time1,y_des1,'--',label='Desired')
plt.xlabel('Time [s]')
plt.ylabel('y [m]')
plt.legend()
plt.subplot(212)
plt.plot(time1,pitch_mocap1,'-',label='Actual')
plt.plot(time1,pitch_des1,'--',label='Desired')
plt.xlabel('Time [s]')
plt.ylabel('Pitch [deg]')
plt.legend()
plt.suptitle('Y-Position Data')

plt.figure(4, figsize=(10, 12))
plt.subplot(211)
plt.plot(time1,x1,'-',label='Actual')
plt.plot(time1,x_des1,'--',label='Desired')
plt.xlabel('Time [s]')
plt.ylabel('x [m]')
plt.legend()
plt.subplot(212)
plt.plot(time1,roll_mocap1,'-',label='Actual')
plt.plot(time1,roll_des1,'--',label='Desired')
plt.xlabel('Time [s]')
plt.ylabel('Roll [deg]')
plt.legend()
plt.suptitle('X-Position Data')

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