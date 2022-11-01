%% IMU Test
% GPS Denied Navigation, AEM 4331 Fall 2022

imu = imu_model();
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
[pos,~,~,acc,angvel,t,~,~] = trajectory2(Fs);

% Sensor response
[accelData, gyroData, magData] = imu(acc, angvel);

subplot(3, 2, 1)
plot(t, accelData)
title('Accelerometer')
xlabel('s')
ylabel('m/s^2')
legend('x','y','z')

subplot(3, 2, 2)
plot(t, gyroData)
title('Gyroscope')
xlabel('s')
ylabel('rad/s')
legend('x','y','z')

subplot(3, 2, 3)
plot(t, magData)
title('Magnetometer')
xlabel('s')
ylabel('uT')
legend('x','y','z')

subplot(3, 2, 4)
plot(t, pos(:,1))
title('x Position')
xlabel('s')
ylabel('m')

subplot(3, 2, 5)
plot(t, pos(:,2))
title('y Position')
xlabel('s')
ylabel('m')

subplot(3, 2, 6)
plot(t, pos(:,3))
title('z Position')
xlabel('s')
ylabel('m')

% plot(t, angvel(:,1), '--', t, gyroData(:,1))
% title('Gyroscope')
% xlabel('s')
% ylabel('rad/s')
% legend('x (truth)','x (gyro)')


