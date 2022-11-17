%===========================================================%
%               gnss_ins_filter_config.m                    %
%                                                           %
%   This m-file contains all the conifiguraiton switches    %
%   for the GNSS/INS filter.                                %
%                                                           %
%   Programmer:     Demoz Gebre-Egziabher                   %
%   Created:        March 26, 2009                          %
%   Last Modified:  March 26, 2009                          %
%                                                           %
%===========================================================%

%   Define start and stop time in MINUTES for playback.  The
%   data provided is from a flighttest of 14 minutes duration.
%   The variables t_start and t_end must define a time interval 
%   between 0 and 14 minutes.  

% t_start = 7;
% t_end = 13.5;
t_start = 0;
t_end = 20; 

%   Configure the Extended Kalman Filter (EKF)

CLOSED_LOOP = 1;        %   If set to 1, a GNSS-aided 
                        %   inertial navigator is simulated.
                        %   If set to 0, the simulation will
                        %   be that of an unaided INS.  
                        
NO_CORIOLIS = 1;        %   If set to 1, coriolis acceleraitons
                        %   are ignored in the time upadate 
                        %   equations. Should be set to 0 when
                        %   using high grade inertial sensors and
                        %   GNSS updates come at a slow rate.
                        
SMALL_PROP_TIME = 1;    %   If set to 1, it means that the time
                        %   between GNSS updates is small and,
                        %   thus, Schuler dynamics can be
                        %   ignored without much consequence.
                        %   That is, the approximation given by
                        %   Equation (6.17) is used for the 
                        %   velocity propagation instead of 
                        %   Equation (6.13)

                                
gnss_update_rate = 1;   %  GNSS measurement update rate in Hz.

%   GPS measurement noise standard deviation

gps_pos_sigma = 3;              %   m (North, East, Down)
gps_vel_sigma = 0.2;            %   m/s (North, East, Donw)

%   IMU output error covariance (Table 1, Chapter 6)

sigma_g_d = 0.3*d2r;                % Standard deviation of correlated gyro bias 
tau_g = 300;                        % Correlation time or time constant of b_{gd}
sigma_g_w = 0.95*d2r;               % Standard deviation of gyro output noise

sigma_a_d = (0.5e-3)*g;             % Standard deviation of Accelerometer Markov Bias
tau_a = 300;                        % Correlation time or time constant of b_{ad}
sigma_a_w = 0.12*g;                 % Standard Deviation of Accelerometer Wide Band Noise
    
        