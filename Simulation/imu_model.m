%% IMU Model
% GPS Denied Navigation, AEM 4331 Fall 2022

%% HG1125CA01 Performance

% Gryo
gyro_bias_repeatability    = 120; % deg/hr
gyro_bias_in_run_stability = 7;   % deg/hr
gyro_arw                   = .3;  % deg/sqrt(hr)

% Convert to param units
gyro_bias_instability = conversions.deg_per_hr_to_rad_per_sec(gyro_bias_in_run_stability); % rad/s
gyro_noise_density    = conversions.deg_per_sqrt_hr_to_rad_per_sqrt_sec(gyro_arw);         % rad/sqrt(s)


% Init params object
gyro_params = gyroparams(...
    'BiasInstability', gyro_bias_instability, ... % rad/s, Instability of the bias offset 
    'NoiseDensity',    gyro_noise_density     ... % rad/sqrt(s), Power spectral density of sensor noise                               
);

% Accelerometer
acc_bias_repeatability     = 1.5; % mg
acc_bias_in_run_stability  = 7;   % mg
acc_vrw                    = .18; % fps/sqrt(hr)

