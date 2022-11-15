%% IMU Test
% GPS Denied Navigation, AEM 4331 Fall 2022

imu   = imu_model();
imuFs = imu.SampleRate;

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
% [pos,~,~,acc,angvel,t,~,~] = trajectory1(imuFs);

% Trajectory 2
[pos,~,vel,acc,angvel,t,~,~] = trajectory2(imuFs);


% IMU
[accelData, gyroData, magData] = imu(acc, angvel);

% GPS
scaledFs = imuFs / gpsFs; % Number of imu cycles per gps update
scaledPos = pos(1:scaledFs:end,:); % Scale down
scaledVel = vel(1:scaledFs:end,:);
scaledT   = t(1:scaledFs:end);
[gps_lla,~,~,~] = gps(scaledPos,scaledVel);
[East,North,Up] = latlon2local(gps_lla(:,1), gps_lla(:,2), gps_lla(:,3), gps.ReferenceLocation);
gps_pos = [North, East, -Up];

gps_err = gps_pos - scaledPos;


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
plot(scaledT, gps_pos(:,1), 'DisplayName','GPS')
title('x Position')
xlabel('s')
ylabel('m')
legend('show','Location','best');

subplot(n_row, n_col, 5)
hold on
plot(t, pos(:,2), 'DisplayName','Truth')
plot(scaledT, gps_pos(:,2), 'DisplayName','GPS')
title('y Position')
xlabel('s')
ylabel('m')
legend('show','Location','best');

subplot(n_row, n_col, 6)
hold on
plot(t, pos(:,3), 'DisplayName','Truth')
plot(scaledT, gps_pos(:,3), 'DisplayName','GPS')
title('z Position')
xlabel('s')
ylabel('m')
legend('show','Location','best');

% plot(t, angvel(:,1), '--', t, gyroData(:,1))
% title('Gyroscope')
% xlabel('s')
% ylabel('rad/s')
% legend('x (truth)','x (gyro)')


