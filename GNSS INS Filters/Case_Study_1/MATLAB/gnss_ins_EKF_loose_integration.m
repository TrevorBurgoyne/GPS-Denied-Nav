%==========================================================================
%                                                                         % 
%                    gnss_ins_EKF_loose_integration.m                     %
%                           (MATLAB Version)                              %
%                                                                         %
%   This m-file simulates a loosly integrated GNSS-INS Extended Kalman    %
%   Filter (EKF) described in Chapter 6.  It operates on data collected   %
%   from a low cost IMU in a playback mode.  This script is for pedagogy  %
%   purposes only.  As such, the attitude parameterization is done using  %
%   Euler angles.  Thus, the singularity at pitch = 90 must be taken into %
%   account when playing back other data sets than the one provided with  %
%   this book.                                                            %
%                                                                         % 
%   Programmers:        Demoz Gebre-Egziabher                             %
%                       Zhiqiang Xing                                     %
%                       Yunfeng Shao                                      %
%   Last Modified:      Juney 26, 2009                                    %
%                                                                         %
%==========================================================================

%   Clear and configure workspace

close all;  clear all;  clc;

if(ispc)                                    %   Path to gnss_ins toolbox
    addpath  '..\..\gnss_ins_functions\';
else
    addpath '../../gnss_ins_functions/';
end

%   Start timer

tic;

%   Load constants

gnss_ins_EKF_constants;

%   Configure simulation

gnss_ins_EKF_config;

%   Load sensor data

gnss_ins_EKF_data_loader;

%   Define place holders for filter variables

eul_ins = zeros(drl,3);         %   Euler angles/Attitude (Roll, Pitch, Yaw)
pos_ins = zeros(drl,3);         %   Position (Geodetic Coordinates)
vel_ins = zeros(drl,3);         %   Velocity (North East Down)
accelBias = zeros(drl,3);       %   Acclerometer bias (m/s/s)
gyroBias = zeros(drl,3);        %   Gyro bias (rad/s)

%   Define place holders for interim/trouble shooting variables

pos_ins_ned = zeros(drl,3);         %   INS/GNSS position in NED coordinates
pos_ins_ecef = zeros(drl,3);        %   INS/GNSS position in ECEF coordinates
gps_pos_ned = zeros(drl,3);         %   GNSS (GPS) position in NED coordinates
gps_pos_ecef = zeros(drl,3);        %   GNSS (GPS) position in ECEF coordinates

posFeedBack = zeros(drl,3);         %   Position corrections
velFeedBack = zeros(drl,3);         %   Velocity corrections
attFeedBack = zeros(drl,3);         %   Attitude/tilt corrections
accelFeedBack = zeros(drl,3);       %   Accel bias corrections
gyroFeedBack = zeros(drl,3);        %   Gyro bias corrections 

%   Define GNSS measurement model

M = 15;                         %   Number of Filter states
H = [eye(6) zeros(6,M-6)];      %   Equation (6.29)

R = diag([ [gps_pos_sigma*ones(1,3)].^2 ...
            [gps_vel_sigma*ones(1,3)].^2  ]);   %   Equation (6.30)
  
%   Determine number of IMU time updates between each 
%   GNSS measurement update

imu_update_rate = 1/(mean(diff(imu(:,1))));             % In units of Hz
tu_per_mu = fix(imu_update_rate/gnss_update_rate);      % Number of time updates
                                                        % per measurement update
                                                        
tu_counter = 0;     %   Time update counter.  After each time update, this
                    %   counter is incremented by one.  When it is greater
                    %   than or equal to tu_per_mu, a measurement update 
                    %   occurs.  After the measurement update, it is reset
                    %   to zero to restart a new counting cycle.

%   Define process noise model
% 
        %   Note that the process noise covariance matrix Q = E{w*w'}
        %   of Equations (6.44) and (6.45) can also be written as Q = G*Rw*G'
        %   where Rw = E{[w_a;w_g;mu_a;mu_g][w_a;w_g;mu_a;mu_g]'}.  Rw is a 
        %   diagonal matrix whose entries are the power spectral densities
        %   given in Table 6.1 and footnote #8 in Chapter 6.  With this in 
        %   mind, note that the G matrix becomes as given below:
                    
Z3 = zeros(3,3);    
I3 = eye(3);

G = [    Z3  Z3  Z3  Z3     ;...
         I3  Z3  Z3  Z3     ;...
         Z3  I3  Z3  Z3     ;...
         Z3  Z3  I3  Z3     ;...
         Z3  Z3  Z3  I3     ];
     
Rw = zeros(12,12);
Rw(1:3,1:3) = (sigma_a_w^2)*eye(3);             %   Accel. uncorrelated noise
Rw(4:6,4:6) = (sigma_g_w^2)*eye(3);             %   Gyro uncorrelated noise
Rw(7:9,7:9) = (2*sigma_a_d^2/tau_a)*eye(3);     %   Accel. correlated noise
Rw(10:12,10:12) = (2*sigma_g_d^2/tau_g)*eye(3); %   Gyro correlated noise                                        
 
%   Establish initial conditions for filter states

pos_ins(1,:) = gps_pos_lla(1,1:3);
vel_ins(1,:) = gps_vel_ned(1,1:3);
eul_ins(1,:) = 0*[roll(1)*d2r,pitch(1)*d2r,yaw(1)*d2r];
accelBias(1,:) = initial_accel_bias;
gyroBias(1,:) = initial_gyro_bias;

%   Establish initial condition for error state covariance

%-----------------------------------------------------------%
%                                                           %
%   This is a 15 state system and the filter state vector   %
%   is described by Equation (6.26).  That is:              %
%                                                           %
%       (1) dp = Position Errors (NED Coordinates)          %
%       (2) dv = Velocity Errors (NED Coordinates)          %
%       (3) dpsi_nb = Platform Tilt/Attitude Errors         %
%       (4) db_a = Accelerometer Bias Estimation Erros      %
%       (5) db_g = Rate Gyro Bias Estimation Errors         %
%                                                           %
%-----------------------------------------------------------%
    
dp = gps_pos_sigma*ones(1,3);
dv = gps_pos_sigma*ones(1,3);
dpsi_nb = 10*d2r*ones(1,3);
db_a = 10*sigma_a_d*ones(1,3); 
db_g = 10*sigma_g_d*ones(1,3); 

dx_o = [dp dv dpsi_nb 10*db_a db_g]';  % Initial navigation error state vector
P = 10*diag(dx_o.^2)';                 % Initial state error covariance

%-------------------------------------------------------------------------%
%                                                                         %
%   Place holders for the covariance history are defined below:           %
%                                                                         %
%    (1) Pp = GNSS/INS blended position solution 1-sigma covariance       %
%    (2) Pv = GNSS/INS blended Velocity solution 1-sigma covariance       %
%    (3) Ppsi = GNSS/INS attitude error (tilt error) 1-sigma covariance   %
%    (4) Pa = Accelerometer bias estimate 1-sigma covariance              %
%    (5) Pg = Rate Gyro bias estimate 1-sigma covariance                  %
%-------------------------------------------------------------------------%

Pp = zeros(drl,3);      Pp(1,:) = diag(P(1:3,1:3))';
Pv = zeros(drl,3);      Pv(1,:) = diag(P(4:6,4:6))';
Ppsi = zeros(drl,3);    Ppsi(1,:) = diag(P(7:9,7:9))';
Pa = zeros(drl,3);      Pa(1,:) = diag(P(10:12,10:12))';
Pg = zeros(drl,3);      Pg(1,:) = diag(P(13:15,13:15))';

%=========================================================================%
%   Main loop.  Process IMU data at a fast rate and use GNSS measurements
%   to update navigation state vector periodically.

%   Initialzie waitbar.  Allows monitoring progress of algorithm

wB = waitbar(0,'Running 3D GNSS/INS Extended Kalman Filter ....');


for k = 2:drl
    
    waitbar(k/drl,wB);
    tau = t(k)-t(k-1);              %   Delta t between IMU samples
    tu_counter = tu_counter + 1;    %   Increment time update counter
    
    %---------------------------------------------------------------------%
    %                       Time Update Equations                         %
    %---------------------------------------------------------------------%
    
    %   Calculate Cbn using attitude from the last time step.
    %   This will be used to propagate the solution to to the next time
    %   step
    
    Cbn = eul2Cbn([eul_ins(k-1,:)]);            %   Equation (6.11)
    
    %   Caclulate the Euler angle rates
    
    Fr = omega2rates([eul_ins(k-1,:)]);         %   Equation (6.8)
    
    %   Sample rate gyros and compute omega_b_nb.  Account for gyro bias
    %   by adding gyrobias estimate from the last measurement update.
    
    omega_b_ib = imu(k-1,2:4)';
    omega_n_ie = earthrate(pos_ins(k-1,1));                 %   Earth rate.  Equation (6.14)
    omega_n_en = navrate(vel_ins(k-1,:),pos_ins(k-1,:)');   %   Transport rate.  Equation (6.15)

    
    omega_b_nb = omega_b_ib - Cbn*(omega_n_en + omega_n_ie) + gyroBias(k-1,:)';  %  Equation (6.9)
    
    %  Estimate attitude at next time step t = t_{k+1}
    
    eul_ins(k,:) = eul_ins(k-1,:) + tau*(Fr*omega_b_nb)';   % Equation (6.10)
   
    %  Sample accelerometers.  Correct accelerometer reading by
    %  adding accelerometer bias estimate from the last measurement update.
    
    f_b = imu(k-1,5:7)' + accelBias(k-1,:)';
    
    %   Compute the local value of gravity expressed in NED ccordinates
    
    g_n = glocal(pos_ins(k-1,1),pos_ins(k-1,3));            %  Equation (6.16)
    
    %   Compute v_n_dot
    
    if (NO_CORIOLIS)
        v_dot = Cbn*f_b + g_n;                  %   Equation (6.17)
    else
        v_dot = Cbn*f_b - cross(2*omega_n_ie + omega_n_en,vel_ins(k-1,:)') + g_n;   % Equation (6.13)
    end
    
    %  Estimate velocity at next time step t = t_{k+1}
    
    vel_ins(k,:) = vel_ins(k-1,:) + tau*v_dot';         %  Equation (6.12)
    
    %   Form the T matrix from Equation (6.4) for position update
    
    [R_N,R_E] = earthrad(pos_ins(k-1,1));
    
    T(1,1) = 1/(R_N + pos_ins(k-1,3));
    T(2,2) = 1/((R_E + pos_ins(k-1,3))*cos(pos_ins(k-1,1)));
    T(3,3) = -1;
    
    pos_ins(k,:) = pos_ins(k-1,:) + (tau*T*vel_ins(k-1,:)')';       %  Equation (6.18 - 6.20)
    
    %   Surpress vertical channel in the absence of GNSS measurements
    
%     vel_ins(k,3) = vel_ins(k-1,3);
%     pos_ins(k,3) = pos_ins(k-1,3);
    
    %   Propagate last accel/gyro bias forward in the absence of GNSS
    %   measurements
    
    gyroBias(k,:) = gyroBias(k-1,:);
    accelBias(k,:) = accelBias(k-1,:);
    
     %    Convert position to other coordinates for later printing.
   
    lat_ref = pos_ins(k,1)*r2d;             %   Set the INS estimated position as reference point
    lon_ref = pos_ins(k,2)*r2d;
    alt_ref = pos_ins(k,3);
       
    pos_ins_ecef(k,:) = wgslla2xyz(lat_ref,lon_ref,alt_ref)';
    pos_ins_ned(k,:) = wgsxyz2ned(pos_ins_ecef(k,:)',lat_ref,lon_ref,0*alt_ref)';
       
    lat_gps = gps_pos_lla(k,1)*r2d;
    lon_gps = gps_pos_lla(k,2)*r2d;
    alt_gps = gps_pos_lla(k,3);
       
    gps_pos_ecef(k,:)= wgslla2xyz(lat_gps,lon_gps,alt_gps)';
    gps_pos_ned(k,:) = wgsxyz2ned(gps_pos_ecef(k,:)',lat_ref,lon_ref,0*alt_ref)';
    
    %----- Propagate navigation state error covariance forward in time ---------%
    
    %---------------------------------------------------------------------%
    %   The linearized error dynamics matrix F and process noise mapping 
    %   matrices are derived.  The linearization is assumed to occur about
    %   the INS estimated states.  This is nothing more than a recasting of
    %   Equations (6.21) - (6.23).  For a detailed derivation of these
    %   equations consult Groves pp 384 - 388 (Ref 3, Chapter 6) or 
    %   Titterton pp 329 - 335 (Ref 6, Chapter 6) 
    %---------------------------------------------------------------------%

    g_mag = norm(g_n);                          %   Local gravity magnitude    
    dp2dp = -skew(omega_n_en);                  %   Pos. to pos. error block
    dp2dv = diag((g_mag/R_0)*[-1 -1 2]);        %   Pos. to vel. error block 
    dv2dv = -skew(2*omega_n_ie + omega_n_en);   %   Vel. to vel. error block
    dpsi2dv = skew(Cbn*f_b);                    %   Att. to vel. error block                                   
    da2dv = Cbn ;                               %   Accel. bias to vel. error block                        
    dpsi2dpsi = -skew(omega_n_ie + omega_n_en); %   Att. to att. error block
    domega2dpsi = -Cbn;                         %   Gyro bias to att. error block    
    da2da = -I3/tau_a;                          %   Accl. bias to accel bias error block
    domega2domega = -I3/tau_g;                  %   Gyro bias to gyro bias error block
    
    %---------------------  Assemble the F and G Matrices  -----------------------% 
    
    if(~SMALL_PROP_TIME)
        
        F = [   dp2dp       I3          Z3          Z3          Z3      ;...
                dp2dv       dv2dv       dpsi2dv     da2dv       Z3      ;...
                Z3          Z3          dpsi2dpsi   Z3          domega2dpsi;...
                Z3          Z3          Z3          da2da       Z3      ;...
                Z3          Z3          Z3          Z3          domega2domega];
            
    elseif(SMALL_PROP_TIME)
        
        dp2dv = diag([ 0 0 2*g_mag/R_0]);
        
        F = [   Z3          I3          Z3          Z3          Z3    ;...
                dp2dv       Z3          dpsi2dv     da2dv       Z3   ;...
                Z3          Z3          Z3          Z3          domega2dpsi  ;...
                Z3          Z3          Z3          dpsi2dpsi   Z3  ;...
                Z3          Z3          Z3          Z3          domega2domega   ;...
                                                                    ];
    end
       
    G(4:6,1:3) = Cbn;
    G(7:9,4:6) = -Cbn; 
    
    %---------- Form Discrete Equivalent of F and G*Rw*G' ----------------% 
    
    PHI = expm(F*tau);
    Q_k = discrete_process_noise(F,G,tau,Rw);
    
    %------------- Propagate Covariance Forward in Time ------------------% 
    
    P = PHI*P*PHI' + Q_k;                         %   Equation (6.27)
    
       
    if(CLOSED_LOOP && tu_counter > tu_per_mu)   %   Check whether to do a
                                                %   measurement update
        
        tu_counter = 0;                         %   Reset time update counter
       
        %   Compute innovations process

        posInnov = pos_ins_ned(k,:)' - gps_pos_ned(k,:)';
        velInnov = vel_ins(k,:)' - gps_vel_ned(k,:)';
        stateInnov = [posInnov; velInnov];

        %   Compute Kalman gain and update state covariance

        Pz = H*P*H' + R;
        L = (P*H')/(Pz);            %   Equation (6.31).  Note that this is 
                                    %   equivalent to but faster than L = P*H'*inv(Pz)       
        P = (eye(15) - L*H)*P;      %   Equation (6.33). Update covariance matrix
        P = 0.5*(P + P');           %   Force covariance matrix symmetry


        stateError = L*stateInnov;   %   Equation (6.32)

        posFeedBack(k,:) = stateError(1:3)';
        velFeedBack(k,:) = stateError(4:6)';
        attFeedBack(k,:) = stateError(7:9)';
        accelFeedBack(k,:) = stateError(10:12)';
        gyroFeedBack(k,:) = stateError(13:15)';

        %   Update position and velocity using computed corrections (Equation 6.34)

        pos_ins(k,1) = pos_ins(k,1) - posFeedBack(k,1)/(R_N+pos_ins(k,3));
        pos_ins(k,2) = pos_ins(k,2) - posFeedBack(k,2)/((R_E+pos_ins(k,3))*cos(pos_ins(k,1)));
        pos_ins(k,3) = gps_pos_lla(k,3);

        vel_ins(k,:) = vel_ins(k,:) - velFeedBack(k,:);
        vel_ins(k,3)=  gps_vel_ned(k,3);
       
        %   Update attitude using computed corrections
       
          % TODO: fix atan error
%         Cbn_minus = Cbn;
%         Cbn_plus = (I3 + skew(attFeedBack(k,:)))*Cbn_minus;     %   Equation (6.35)
%         eul_ins(k,:) = [Cbn2eul(Cbn_plus)]';                    %   Equation (6.36) - Equation (6.38)
        
        %   Update sensor bias estimates
        
        accelBias(k,:) = accelBias(k-1,:) - accelFeedBack(k,:);
        gyroBias(k,:) = gyroBias(k-1,:) - gyroFeedBack(k,:);
        
    end                         %   End of measurement update
    
    %   Save covariance history
    
    Pp(k,:) = diag(P(1:3,1:3))';
    Pv(k,:) = diag(P(4:6,4:6))';
    Ppsi(k,:) = diag(P(7:9,7:9))';
    Pa(k,:) = diag(P(10:12,10:12))';
    Pg(k,:) = diag(P(13:15,13:15))';
 
end                    
                    
%   Close waitbar and stop timer

close(wB);
toc; 
     
%   Plot results

plot_EKF_output;
               
                    
                    
        