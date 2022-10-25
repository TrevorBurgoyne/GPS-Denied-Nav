%% IMU Model
% GPS Denied Navigation, AEM 4331 Fall 2022
% HG1125CA01 Performance

%% Gryo
gyro_bias_repeatability    = 120; % deg/hr
gyro_bias_in_run_stability = 7;   % deg/hr
gyro_arw                   = .3;  % deg/sqrt(hr)

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

