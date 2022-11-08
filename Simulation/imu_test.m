%% IMU Test
% GPS Denied Navigation, AEM 4331 Fall 2022

imu = imu_model();
gps = gps_model();
Fs = imu.SampleRate;

% Spin
% N = 1000;
% Fc = .25; % sinusoidal frequency
% t = (0:(1/imu.SampleRate):((N-1)/imu.SampleRate)).';
% acc = zeros(N,3);
% angvel = zeros(N,3);
% angvel(:,1) = sin(2*pi*Fc*t);



% Trajectory 1
% [pos,~,~,acc,angvel,t,~,~] = trajectory1(Fs);

% Trajectory 2
[pos,~,vel,acc,angvel,t,~,~] = trajectory2(Fs);
% [lat, lon] = local2latlon(pos(:,1), pos(:,2), pos(:,3), gps.ReferenceLocation);
% lla = [lat lon pos(:,3)]; % Lat, Lon, Alt

% Sensor response
[accelData, gyroData, magData] = imu(acc, angvel);
[gps_lla,~,~,~] = gps(pos,vel);
[East,North,Up] = latlon2local(gps_lla(:,1), gps_lla(:,2), gps_lla(:,3), gps.ReferenceLocation);
gps_pos = [North, East, -Up];

err = gps_pos - pos;


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
plot(t, gps_pos(:,1), 'DisplayName','GPS')
title('x Position')
xlabel('s')
ylabel('m')
legend('show','Location','best');

subplot(n_row, n_col, 5)
hold on
plot(t, pos(:,2), 'DisplayName','Truth')
plot(t, gps_pos(:,2), 'DisplayName','GPS')
title('y Position')
xlabel('s')
ylabel('m')
legend('show','Location','best');

subplot(n_row, n_col, 6)
hold on
plot(t, pos(:,3), 'DisplayName','Truth')
plot(t, gps_pos(:,3), 'DisplayName','GPS')
title('z Position')
xlabel('s')
ylabel('m')
legend('show','Location','best');

% plot(t, angvel(:,1), '--', t, gyroData(:,1))
% title('Gyroscope')
% xlabel('s')
% ylabel('rad/s')
% legend('x (truth)','x (gyro)')


