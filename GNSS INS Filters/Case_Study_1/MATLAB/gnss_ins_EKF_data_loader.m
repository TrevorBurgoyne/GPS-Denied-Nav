%=======================================================%
%               gnss_ins_data_loader.m                  %
%                   (MATLAB Version)                    %
%                                                       %
%   This m-file loads in data that will be played back  %
%   in the GNSS/INS EKF.  The data was collected from   %
%   flight test onboard a small hand launched aerial    %
%   vehicle and is contained in the file named          %
%   flight_test_data_set.mat.                           %
%                                                       %
%   Programmer:     Demoz Gebre-Egziabher               %
%   Created:        March 26, 2009                      %
%   Last Modified:  June 26, 2009                       %
%                                                       %
%=======================================================%


load flight_data.mat;        %   Sensor data data

%   Establish initial value for inertial sensor biases
% imu = [t, gyro_x, gyro_y, gyro_z, acc_x, acc_y, acc_z]
initial_gyro_bias = -mean(imu(1:1000,2:4));
initial_accel_bias = -(mean(imu(1:1000,5:7)) + [0 0 g]);

%   Find subset of data that falls between t_start and t_end

idx = find(t >= t_start*60);
k_s = idx(1);
idx = find(t >= t_end*60);
k_e = idx(1);

idx = [k_s:k_e];
drl = length(idx);

%   Extract IMU, GPS and attitude data that falls between t_start and t_end

t_temp = t(idx) - ones(drl,1)*t(1);
imu_temp = imu(idx,:);
gps_pos_lla_temp = gps_pos_lla(idx,:);
gps_vel_ned_temp = gps_vel_ned(idx,:);
roll_temp = roll(idx);
pitch_temp = pitch(idx);
yaw_temp = yaw(idx);

%   Clear extraneous data

clear t imu gps_pos_lla gps_vel_ned roll pitch yaw k_s k_e idx

%   Repackage data into original name variables

t = t_temp;
imu = imu_temp;
gps_pos_lla = gps_pos_lla_temp;
gps_vel_ned = gps_vel_ned_temp;
roll = roll_temp;
pitch = pitch_temp;
yaw = yaw_temp;

clear t_temp imu_temp gps_pos_lla_temp gps_vel_ned_temp roll_temp pitch_temp yaw_temp

