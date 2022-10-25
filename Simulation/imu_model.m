%% IMU Model
% GPS Denied Navigation, AEM 4331 Fall 2022
% HG1125CA01 Performance

%% Gryo
gyro_bias_repeatability    = 120; % deg/hr
gyro_bias_in_run_stability = 7;   % deg/hr
gyro_arw                   = 0.3;  % deg/sqrt(hr)

% Convert to param units
gyro_bias_instability = conversions.deg_per_hr_to_rad_per_sec(gyro_bias_in_run_stability); % rad/s
gyro_noise_density    = conversions.deg_per_sqrt_hr_to_rad_per_sqrt_sec(gyro_arw);         % rad/sqrt(s)


% Init gyroparams object
gyro_params = gyroparams(...
    'BiasInstability', gyro_bias_instability, ... % rad/s, Instability of the bias offset 
    'NoiseDensity',    gyro_noise_density     ... % rad/sqrt(s), Power spectral density of sensor noise                               
);

%% Accelerometer
acc_bias_repeatability     = 1.5; % mg
acc_bias_in_run_stability  = 7;   % mg
acc_vrw                    = .18; % fps/sqrt(hr)

% Convert to param units
acc_bias_instability = conversions.mg_to_m_per_s_squared(acc_bias_in_run_stability);   % m/s^2
acc_noise_density    = conversions.fps_per_sqrt_hr_to_m_per_sec_per_sqrt_sec(acc_vrw); % m/s/sqrt(s) or m/s^2/sqrt(Hz)

% Init accelparams object
accel_params = accelparams(...
    'BiasInstability', acc_bias_instability, ... % m/s^2, Instability of the bias offset 
    'NoiseDensity',    acc_noise_density     ... % m/s^2/sqrt(Hz), Power spectral density of sensor noise 
);


%% IMU
N = 1000;
Fs = 100; % Hz, sampling rate
Fc = .25; % sinusoidal frequency

t = (0:(1/Fs):((N-1)/Fs)).';
acc = zeros(N,3);
angvel = zeros(N,3);
angvel(:,1) = sin(2*pi*Fc*t);

imu = imuSensor(...
  'accel-gyro-mag', ...
  'ReferenceFrame', 'NED', ...
  'SampleRate', Fs, ...
  'Gyroscope', gyro_params, ...
  'Accelerometer', accel_params ...
);

[accelData, gyroData, magData] = imu(acc, angvel);

% subplot(3, 1, 1)
% plot(t, accelData)
% title('Accelerometer')
% xlabel('s')
% ylabel('m/s^2')
% legend('x','y','z')
% 
% subplot(3, 1, 2)
% plot(t, gyroData)
% title('Gyroscope')
% xlabel('s')
% ylabel('rad/s')
% legend('x','y','z')
% 
% subplot(3, 1, 3)
% plot(t, magData)
% title('Magnetometer')
% xlabel('s')
% ylabel('uT')
% legend('x','y','z')

plot(t, angvel(:,1), '--', t, gyroData(:,1))
title('Gyroscope')
xlabel('s')
ylabel('rad/s')
legend('x (truth)','x (gyro)')


