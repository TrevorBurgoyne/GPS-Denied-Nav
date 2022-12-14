%% IMU Test
% GPS Denied Navigation, AEM 4331 Fall 2022

imu   = imu_model();
% imuFs = imu.SampleRate;
imuFs = 50; % Hz, to ease computation time

gps   = gps_model();
gpsFs = gps.SampleRate;

% Spin
% N = 1000;
% Fc = .25; % sinusoidal frequency
% t = (0:(1/imu.SampleRate):((N-1)/imu.SampleRate)).';
% acc = zeros(N,3);
% angvel = zeros(N,3);
% angvel(:,1) = sin(2*pi*Fc*t);



% Trajectory 1
% [pos,orient,vel,acc,angvel,t,~,~] = trajectory1(imuFs);
% eul = quat2eul(orient); % rad, [Roll, Pitch, Yaw]
% save_name = 'Trajectory1';

% Trajectory 2
[pos,orient,vel,acc,angvel,t,~,~] = trajectory2(imuFs);
eul = quat2eul(orient); % rad, [Roll, Pitch, Yaw]
save_name = 'Trajectory2';

% Speed
speeds = vecnorm(vel');
avg_speed = mean(speeds);
min_speed = min(speeds);
max_speed = max(speeds);

% IMU
[accelData, gyroData, magData] = imu(acc, angvel);

[~,~,~,init_acc,init_angvel,init_t,~,~] = init_trajectory(imuFs);
[init_accelData, init_gyroData, init_magData] = imu(init_acc, init_angvel);

% GPS

% scaledFs = imuFs / gpsFs; % Number of imu cycles per gps update
% scaledPos = pos(1:scaledFs:end,:); % Scale down
% scaledVel = vel(1:scaledFs:end,:);
% scaledT   = t(1:scaledFs:end);
% [gps_lla,~,~,~] = gps(scaledPos,scaledVel);

[gps_pos_lla,gps_vel_ned,~,~] = gps(pos,vel);
[East,North,Up] = latlon2local(gps_pos_lla(:,1), gps_pos_lla(:,2), gps_pos_lla(:,3), gps.ReferenceLocation);
gps_pos_local = [North, East, -Up];

% gps_err = gps_pos - scaledPos;

%% Save flight data
% set up imu array [t, gyro_x, gyro_y, gyro_z, acc_x, acc_y, acc_z]
imu = zeros(length(t), 7);
imu(:,1) = t;
imu(:,2:4) = gyroData;
imu(:,5:7) = accelData;
% imu init
imu_init = zeros(length(init_t), 7);
imu_init(:,1) = init_t;
imu_init(:,2:4) = init_gyroData;
imu_init(:,5:7) = init_accelData;
% convert gps pos to rad
gps_pos_lla(:,1:2) = deg2rad(gps_pos_lla(:,1:2));
% roll, pitch, yaw
roll = eul(:,1);
pitch = eul(:,2);
yaw = eul(:,3);
% invert t
t = t';
% true pos (lla in rad)
[lat, lon, alt] = local2latlon(pos(:,1), pos(:,2), pos(:,3), gps.ReferenceLocation);
truth_pos_lla = zeros(length(t),3); 
truth_pos_lla(:,1) = deg2rad(lat);
truth_pos_lla(:,2) = deg2rad(lon);
truth_pos_lla(:,3) = alt;
truth_pos = pos;
save(save_name,...
    'imu_init',...
    'imu',...
    'gps',...
    'gps_pos_lla',...
    'gps_pos_local',...
    'gps_vel_ned',...
    'roll',...
    'pitch',...
    'yaw',...
    't',...
    'truth_pos_lla',...
    'truth_pos',...
    'vel'...
);
%% Plot

n_row = 3;
n_col = 2;
subplot(n_row, n_col, 1)
plot(t, accelData)
title('Accelerometer')
xlabel('s')
ylabel('m/s^2')
legend('x','y','z')

subplot(n_row, n_col, 2)
plot(t, gyroData)
title('Gyroscope')
xlabel('s')
ylabel('rad/s')
legend('x','y','z')

subplot(n_row, n_col, 3)
plot(t, magData)
title('Magnetometer')
xlabel('s')
ylabel('uT')
legend('x','y','z')

subplot(n_row, n_col, 4)
hold on
plot(t, pos(:,1), 'DisplayName','Truth')
plot(t, gps_pos_local(:,1), 'DisplayName','GPS')
title('x Position')
xlabel('s')
ylabel('m')
legend('show','Location','best');

subplot(n_row, n_col, 5)
hold on
plot(t, pos(:,2), 'DisplayName','Truth')
plot(t, gps_pos_local(:,2), 'DisplayName','GPS')
title('y Position')
xlabel('s')
ylabel('m')
legend('show','Location','best');

subplot(n_row, n_col, 6)
hold on
plot(t, pos(:,3), 'DisplayName','Truth')
plot(t, gps_pos_local(:,3), 'DisplayName','GPS')
title('z Position')
xlabel('s')
ylabel('m')
legend('show','Location','best');

% plot(t, angvel(:,1), '--', t, gyroData(:,1))
% title('Gyroscope')
% xlabel('s')
% ylabel('rad/s')
% legend('x (truth)','x (gyro)')


