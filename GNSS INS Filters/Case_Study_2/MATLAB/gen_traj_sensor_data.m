            function gen_traj_sensor_data(TRAJECTORY,SAVE_FLAG)
%=====================================================================%
%           function gen_traj_sensor_data(TRAJECTORY,SAVE_FLAG)
%
%   This generates the sensor and trajectory data for simulation
%   used in Cast Study 2 of Chapter 7.
%
%  Programmer:              Demoz Gebre-Egziabher
%  Created:                 March 15, 2007
%  Last Modified:           June 17, 2009 
%
%=====================================================================%

close all;

%==================================================================================%
% (a)                         Input Argument Check                                 %
%==================================================================================%

if (nargin < 1)
    error('Trajectory not specified.  Terminating...');
end

%==================================================================================%
% (b)                      Define Conversion Constants                             %
%==================================================================================%

d2r = pi/180;
r2d = 1/d2r;
g = 9.81;

%==================================================================================%
% (c)                      Define Simulation Parameters                            %
%==================================================================================%

dt = 0.1;               %   Sampling Period
Tf = 3;                 %   Length of Simulation (in minutes)
t = [0:dt:Tf*60]';      %   Time Vector (in seconds)
drl = length(t);        %   Data Record Length


f_psi = 1e-2;           %   Frequency of heading change (Hz)
w_psi = 0*pi/2;         %   Initial phase offset of heading change

vel_har_fac = 0.1;      %   Velocity harmonic pertubation factor: It 
                        %   specifies magnitude of the velocity oscillations
                        %   about the mean (or steady state) value 
                        
f_v = 0.0750;           %   Velocity pertubation frequency (Hz)
w_v = pi/2;             %   Initial phase offset for velocity pertubation

if(strcmp(TRAJECTORY,'A'));         %   Traveling North at a constant speed 
                                    %   and sinusoidally varying heading
    
    max_psi = 45;       %   Maximum heading (yaw) angles in degrees
    max_speed = 30;     %   Maximum speed during trajectory (m/s)
    
    v = max_speed*ones(drl,1);
    vdot = 0;
    
elseif(strcmp(TRAJECTORY,'B'));     %   Traveling North at a constant speed 
                                    %   and heading
                                    
    max_psi = 0;        %   Maximum heading (yaw) angles in degrees
    max_speed = 30;     %   Maximum speed during trajectory (m/s)
    
    v = max_speed*ones(drl,1);
    vdot = 0;
    
elseif(strcmp(TRAJECTORY,'C'));     %   Traveling North with a sinusoidally 
                                    %   varying heading and speed
    
    max_psi = 45;           %   Maximum heading (yaw) angles in degrees
    nominal_speed = 30;     %   Average speed  Vx_m = 30;            
    
    v = nominal_speed*(1 + vel_har_fac*sin(2*pi*f_v*t + w_v));
    vdot = nominal_speed*vel_har_fac*2*pi*f_v*cos(2*pi*f_v*t + w_v);
    
elseif(strcmp(TRAJECTORY,'D'));     %   Traveling North with a sinusoidally 
                                    %   varying speed
    
    max_psi = 0;            %   Maximum heading (yaw) angles in degrees
    nominal_speed = 30;     %   Average speed  Vx_m = 30;            
    
    v = nominal_speed*(1 + vel_har_fac*sin(2*pi*f_v*t + w_v));
    vdot = nominal_speed*vel_har_fac*2*pi*f_v*cos(2*pi*f_v*t + w_v);
 
else
    disp('Undefined trajectory specified....Terminating');
    return;
end

%==================================================================================%
% (d)                      Begin to Generate Data                                  %
%==================================================================================%

%   (d.1)   Generate Heading and Heading Rate Histories

psi_true = d2r*(max_psi*sin(2*pi*f_psi*t + w_psi));                         %  True Heading History
psidot = d2r*(2*pi*f_psi*max_psi*cos(2*pi*f_psi*t + w_psi));        %  True Heading Rate History

%  (d.2)    East-West Velocity, Acceleration and Position History

v_E = v.*sin(psi_true);
v_E_dot = vdot.*sin(psi_true) + v.*cos(psi_true).*psidot;

p_E = zeros(drl,1);
p_E(2:end) = p_E(1) + dt*cumsum(v_E(1:end-1));

%  (d.3)    North-South Position, Velocity and Acceleration History

v_N = v.*cos(psi_true);
v_N_dot = vdot.*cos(psi_true) - v.*sin(psi_true).*psidot;

p_N = zeros(drl,1);
p_N(2:end) = p_N(1) + dt*cumsum(v_N(1:end-1));

%==================================================================================%
% (d)                      Generate Clean IMU Outputs                              %
%==================================================================================%

xddot = zeros(drl,1);           %   x-accelerations in the body frame           
yddot = zeros(drl,1);           %   y-accelerations in the body frame

H = waitbar(0,'Generating Inertial Sensor Data ... ');
for k=1:drl
    Cn2b = [cos(psi_true(k)) sin(psi_true(k));-sin(psi_true(k)) cos(psi_true(k))];
    %   Note:  Cn2b is the navigation-to-body transformation matrix %
    aT = Cn2b*[v_N_dot(k);v_E_dot(k)];  %   True acceleration in the body frame
    xddot(k) = aT(1);
    yddot(k) = aT(2);
    waitbar(k/drl,H);
end
close(H)

%==================================================================================%
% (e)                   Corrupt the Clean IMU Outputs                              %
%==================================================================================%

%   (e.1)   Generate Null Shift

b_asx = 0.1*g;           %   x-accelerometer null shift (m/s/s)
b_asy = 0.1*g;           %   y-accelerometer null shift (m/s/s)

b_gs = 0.5*d2r;          %   Rate gyro null shift (radians/sec).

%   (e.2)   Define Correlated Bias Parameters

tau_a = 300;                %   Correlation time for accelerometer errors

sigma_adx = 0.001*g;        %   Standard deviation of x-accelerometer 
                            %   correlated bias
b_adx = markov_noise(sigma_adx,tau_a,dt,t(end)); 

sigma_ady = 0.002*g;        %   Standard deviation of x-accelerometer 
                            %   correlated bias                            
b_ady = markov_noise(sigma_ady,tau_a,dt,t(end)); 

tau_g = 300;                %   Correlation time for gyro errors
sigma_gd = 0.05*d2r;        %   Standard deviatio of gyro correlated bias
b_gd = markov_noise(sigma_gd,tau_g,dt,t(end));

%   (e.3)   Define Wide Band Noise Parameters

sigma_wa = 0.001*g;         %   Accelerometer output noise standard 
                            %   deviation
w_a = sigma_wa*randn(drl,1);                                                        
                            
sigma_wg = 0.001*d2r;       %   Rate gyro output noise standard deviation
w_g = sigma_wg*randn(drl,1);                                                        

%   (e.4)   Combined Sensor Errors

b_ax = b_asx + b_adx + w_a;
b_ay = b_asy + b_ady + w_a;
b_g = b_gs + b_gd + w_g;

%   (e.4)   Generate Inertial Sensor Outputs

f_x = xddot + b_ax;
f_y = yddot + b_ay;
omega_z = psidot + b_g;

%==================================================================================%
% (f)              Generate Magnetometer/Compass Outputs                           %
%==================================================================================%

%   Note:  The magnetometer/compass errors are assumed to be uncorrelated
%   white noise sequences.  This is a gross idealization.  Realistic error
%   models for these sensors include several correlated terms.
%   Incorporating such errors here will detract from the focus of this 
%   m-script. For more more detailed and realistic magnetometer error 
%   models, consult the following:
%   [1] D. Gebre-Egziabher "Magnetometer Auto-Calibration Leveraging 
%       Measurement Locus Constraints,” AIAA Journal of Aircraft, Vol. 44, 
%       No. 4.  pp. 1361 – 1368.
%   [2] D. Gebre-Egziabher, G. H. Elkaim, J. D. Powell and B.W.
%       Parkinson, "Calibration of Strapdown Magnetometers in the Magnetic 
%       Field Domain," Journal of Aerospace Engineering, Vol. 19, 
%       No. 2, 2006. Pp. 6 - 16.

compass_noise = 0.25*d2r;       %   Standard deviation of compass noise
psi_compass = psi_true + compass_noise*randn(drl,1);

%==================================================================================%
% (f)                   Generate GNSS measurement errors                           %
%==================================================================================%

%   Note:  The GNSS errors here are assumd to be uncorrelated.  For the level
%   accuracy we are dealing with here this is acceptable. 

GNSS_vel_noise = 0.1;           %   m/s
GNSS_pos_noise = 1.0;           %   m

v_N_GNSS = v_N + GNSS_vel_noise*randn(drl,1);
v_E_GNSS = v_E + GNSS_vel_noise*randn(drl,1);

p_N_GNSS = p_N + GNSS_pos_noise*randn(drl,1);
p_E_GNSS = p_E + GNSS_pos_noise*randn(drl,1);


%==================================================================================%
% (h)                                Save Data                                     %
%==================================================================================%

if (SAVE_FLAG)
    
    if(ispc)
        pathName = '.\New_Data\';
    else
        pathName = './New_Data/';
    end

    fileName = ['TRAJECTORY_',TRAJECTORY,'.mat'];
    path_n_file = [pathName,fileName];

    eval(['save ',path_n_file,' ','t dt p_N_GNSS p_E_GNSS v_N_GNSS v_E_GNSS psi_compass']);
    eval(['save ',path_n_file,' ','xddot yddot psidot p_N p_E v_N v_E psi_true -append;']); 
    eval(['save ',path_n_file,' ','f_x f_y omega_z b_ax b_ay b_g w_a w_g -append;']);
    eval(['save ',path_n_file,' ','b_asx b_asy b_adx b_ady tau_a b_gs b_gd tau_g -append;']); 
    eval(['save ',path_n_file,' ','compass_noise GNSS_vel_noise GNSS_pos_noise -append;']);
    eval(['save ',path_n_file,' ','sigma_adx sigma_ady sigma_gd sigma_wa sigma_wg -append;']);
    
end

%==================================================================================%
% (i)                       Plot Data  for Inspection                              %
%==================================================================================%


% figure(gcf);
figure()
h1 = plot(p_E,p_N);grid on;set(h1,'LineWidth',2);
ylabel('North Position (m)');
xlabel('East Position (m)');
axis('equal');
title(['Ground Path of Vehicle for Trajectory ',TRAJECTORY,'.']);

% figure(gcf+1);
figure()
h1 = plot(t,r2d*psi_true);grid on;set(h1,'LineWidth',2);
ylabel('Heading/Yaw ( \psi_{nb} ) in deg');
xlabel('Time (sec)');
title(['Vehicle Heading History for Trajectory ',TRAJECTORY,'.']);

% figure(gcf+1)
figure()
subplot(311)
title(['Velocity and Heading History. Trajectory ',TRAJECTORY,'.']);
h1 = plot(t,v_N,'r-');grid on;ylabel('v_N (m/s)');
set(h1,'LineWidth',2);
subplot(312)
h2 = plot(t,v_E,'r-');grid on;ylabel('v_E (m/s)');
set(h2,'LineWidth',2);
subplot(313)
h3 = plot(t,r2d*psi_true,'r-');grid on;ylabel(' \psi_{nb} (deg)');
xlabel('Time (sec)');
set(h3,'LineWidth',2);

% figure(gcf+1)
figure()
title(['Speed History. Trajectory ',TRAJECTORY,'.']);
h1 = plot(t,sqrt(v_N.^2 + v_E.^2),'r-');grid on;
xlabel('Time (sec)');ylabel('v (m/s)');
set(h1,'LineWidth',2);

% figure(gcf+1)
figure()
subplot(311)
title(['Acceleration and Heading History. Trajectory ',TRAJECTORY,'.']);
h1 = plot(t,xddot,'r-');grid on;ylabel('a_N (m/s^2)');
set(h1,'LineWidth',2);
subplot(312)
h2 = plot(t,yddot,'r-');grid on;ylabel('a_E (m/s^2)');
set(h2,'LineWidth',2);
subplot(313)
h3 = plot(t,r2d*psi_true,'r-');grid on;ylabel('\Psi (deg)');
xlabel('Time (sec)');
set(h3,'LineWidth',2);

%==========================================================================
%==========================================================================
            function x = markov_noise(sigma,tau,Ts,Tf)
%----------------------------------------------------------------
%           function x = markov_noise(sigma,tau,Ts,Tf)
%
%   This function generates a vector containing the output sequence of 
%   a discrete time first order Gauss-Markov process with variance 
%   sigma*sigma, and a time constant tau.  The sampling time for the 
%   discrete process is Ts and the output generated is for a time period 
%   0 <= t <= Tf.
%
%   Demoz Gebre-Egziabher 9/3/98
%---------------------------------------------------------------------

a = -1/tau;;b = 1;c = 1;d = 0;
Q = 2*sigma*sigma/tau;              %   Driving Noise White Power Spectral Density
Qd = discrete_process_noise(a,b,Ts,Q);
Csystem = ss(a,b,c,d);
Dsystem = c2d(Csystem,Ts);
[ad,bd,cdd,dd] = ssdata(Dsystem);
sigmaU = sqrt(Qd);
u = sigmaU*randn(length([0:Ts:Tf]),1);
x = zeros(length(u),1);

for k=2:length(u)
    x(k,1) = ad*x(k-1,1) + u(k-1);    
end;

%***********************************************************************