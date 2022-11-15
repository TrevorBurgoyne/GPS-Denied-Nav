     function gnss_ins_EKF_2D(TRAJECTORY,FILTER_ARCHITECTURE,LINEARIZE_ABT_TRUE,...
                                    FILTER_FLAG, SAVE_FLAG)
%==================================================================================%
%    function gnss_ins_EKF_2D(TRAJECTORY,FILTER_ARCHITECTURE,LINEARIZATION_FLAG,...
%                                    FILTER_FLAG, SAVE_FLAG)
%
%   This is an observability study of a 2-D aided INS.  That is, the INS is mecha-
%   nized with two accelerometers and one gyro to measure acceletion in the plane
%   level to the locally tangent frame.  When the INS is heading true north, the
%   accelerometers measure the north and east accelerations respectively while the
%   rate gyro measures rotation about the vertical axis.
%
%   Aiding comes from a GPS like system (or GPS), which provides position and velocity
%   information.  Heading comes from a magnetometer pair or a multi-antenna GPS 
%   attitude determination system.  
%
%   The unknown navigation error state vector, dx, is 
%   given by the following:
%
%                       dx = [dpn dpe dvn dve dpsia dfx dfy dw]'
%
%   The elements of dx are defined as follows:
%
%   dpn = North-South Position Error (m)
%   dpe = East-West Position Error (m)
%   dvn = North-South Velocity Errror (m/s)
%   dve = East-West Velocity Error (m/s)
%   dpsia = Heading Error (radians)
%   dfx = x-axis accelerometer measurement error (m/s/s)
%   dfy = y-axis accelerometer measurement error (m/s/s)
%   dw = Rate gyro measurement error (rad/s)
%
%   The script below mechanizes an integration filter and generates
%   navigation state estimates as well as sensor error estimates.
%
%   Programmer:         Demoz Gebre-Egziabher
%   Last Modified:      March 15, 2007
%   Last Modified:      Feb 11, 2007
%
%==================================================================================%


%==================================================================================%
% (a)                          Clear Up Work Space                                 %
%==================================================================================%

close all; clc;

%==================================================================================%
% (b)                     Load in Appropriate Trajectory History                   %
%==================================================================================%

if(strcmp(TRAJECTORY,'A'));
    if(ispc)
        load .\New_Data\TRAJECTORY_A.mat
    else
        load ./New_Data/TRAJECTORY_A.mat
    end
    %  heading North at a constant speed and sinusoidally varying heading   
elseif(strcmp(TRAJECTORY,'B'))
    if(ispc)
        load .\New_Data\TRAJECTORY_B.mat
    else
        load ./New_Data/TRAJECTORY_B.mat
    end    %  heading North at constant speed and heading 
elseif(strcmp(TRAJECTORY,'C'))
    if(ispc)
        load .\New_Data\TRAJECTORY_C.mat
    else
        load ./New_Data/TRAJECTORY_C.mat
    end    %  heading North at a sinusoidally varying heading and speed 
elseif(strcmp(TRAJECTORY,'D'))
    if(ispc)
        load .\New_Data\TRAJECTORY_D.mat
    else
        load ./New_Data/TRAJECTORY_D.mat
    end    %  heading North at constant heading but sinusoidally varying speed
else
    error('You did not select a state trajectory history.  Terminating...');
end

%==================================================================================%
% (c)                 Select the Appropriate Filter Architecture                   %
%==================================================================================%

%==================================================================================%
% (g)               Generate Measurement and Process Noise Matrices                %
%==================================================================================%

if(strcmp(FILTER_ARCHITECTURE,'PVH'))           %   Position, Velocity and Heading Aiding
    H = [eye(5) zeros(5,3)];
    R = diag([GNSS_pos_noise GNSS_pos_noise GNSS_vel_noise GNSS_vel_noise compass_noise].^2);
elseif(strcmp(FILTER_ARCHITECTURE,'PV'))        %   Position and Velocity
    H = [eye(4) zeros(4,4)];
    R = diag([GNSS_pos_noise GNSS_pos_noise GNSS_vel_noise GNSS_vel_noise].^2);
elseif(strcmp(FILTER_ARCHITECTURE,'PH'));       %   Position and Heading aiding
    H = [     eye(2)    zeros(2,2)      zeros(2,1)       zeros(2,3)   ;...
            zeros(1,2)  zeros(1,2)          1            zeros(1,3)  ];
    R = diag([GNSS_pos_noise GNSS_pos_noise compass_noise].^2);
elseif(strcmp(FILTER_ARCHITECTURE,'P'));        %   Position only aiding
    H = [eye(2) zeros(2,6)];
    R = (GNSS_pos_noise^2)*eye(2);
else
    error('You did not select an Estimator Architecture.  Terminating ...');
end

%   Establish power spectral densities for the process noise

psd_w_u = [sigma_wa^2 sigma_wa^2 sigma_wg^2];
psd_w_c = 2*[(sigma_adx^2)/tau_a (sigma_ady^2)/tau_a sigma_gd^2/tau_g];
PSD_w = diag([psd_w_u psd_w_c]);

%==================================================================================%
% (d)                Initialize State Error Covariance Matrix                      %
%==================================================================================%

pos_vel_uncertain = [GNSS_pos_noise*ones(1,2) GNSS_vel_noise*ones(1,2) compass_noise].^2;
sensor_error_uncertain = [b_asx b_asy b_gs].^2;
P = diag([pos_vel_uncertain sensor_error_uncertain]);      

%==================================================================================%
% (d)     Define Constants, Place Holders and Establish Initial Conditions         %
%==================================================================================%

d2r = pi/180;
r2d = 1/d2r;
g = 9.81;

drl = length(t);                                %  Data record length

v_NC = zeros(drl,1);    v_NC(1) = v_N(1);       %   "C" is for "computed"
v_EC = zeros(drl,1);    v_EC(1) = v_E(1);       %   That is, these are what
                                                %   EKF outputs as
                                                %   estimates

p_NC = zeros(drl,1);        p_NC(1) = p_N(1);
p_EC = zeros(drl,1);        p_EC(1) = p_E(1);

psiC = zeros(drl,1);        psiC(1) = psi_true(1);

b_axC = zeros(drl,1);           %  These are the estimates that the program will generate.
b_ayC = zeros(drl,1);
b_gC = zeros(drl,1);

%==================================================================================%
% (d)         Establish Initial State Covariance and Filter Update Rate            % 
%==================================================================================%

Ts = 1.0;                   %  Measurement Update Interval in Seconds
tu_per_mu = fix(Ts/dt);     %  Number of Time Update Steps per Measurement Update
num_of_tu = 0;              %  Number of Time Updates since the Last Measurment Update

%==================================================================================%
% (e)                      Main Loop Computation Loop                              % 
%==================================================================================%

for k = 2:drl
    
    if (LINEARIZE_ABT_TRUE)
        psiL = psi_true(k-1);       % "L" is for linearize
    else
        psiL = psiC(k-1);
    end
    
%   Form the time varying dynamics matrix, F    

    F = [0 0 1 0 0         0              0         0   ;...
         0 0 0 1 0         0              0         0   ;...
         0 0 0 0 0    cos(psiL)     -sin(psiL)      0   ;...
         0 0 0 0 0    sin(psiL)     cos(psiL)       0   ;...
         0 0 0 0 0         0              0         1   ;...
         0 0 0 0 0     -1/tau_a           0         0   ;...
         0 0 0 0 0         0          -1/tau_a      0   ;...
         0 0 0 0 0         0              0     -1/tau_g   ];
     
     if (LINEARIZE_ABT_TRUE)
         F(3,5) = -sin(psiL)*xddot(k-1) - cos(psiL)*yddot(k-1);
         F(4,5) =  cos(psiL)*xddot(k-1) - sin(psiL)*yddot(k-1);
     else
         a_xC = f_x(k-1) + b_axC(k-1);
         a_yC = f_y(k-1) + b_ayC(k-1);
         F(3,5) = -sin(psiL)*a_xC - cos(psiL)*a_yC;
         F(4,5) =  cos(psiL)*a_xC - sin(psiL)*a_yC;
     end
 
%   Form the time varying process noise mapping matrix, G

    G  = [       0               0           0   0  0  0;...
                 0               0           0   0  0  0;...
          cos(psiL)         -sin(psiL)       0   0  0  0;...
          sin(psiL)         cos(psiL)        0   0  0  0;...
                 0               0           1   0  0  0;...
                 0               0           0   1  0  0;...
                 0               0           0   0  1  0;...                
                 0               0           0   0  0  1];

%   Form discrete equivalent of the F and G*Rw*G' Matrices
         
    PHI = expm(dt*F);
    Q_k = discrete_process_noise(F,G,dt,PSD_w);

%   Propagate the State Error Covariance Matrix Forward in Time
    
    P = PHI*P*PHI' + Q_k;
    
%   Time Update of the Navigation State Vector
         
    psiC(k) = psiC(k-1) + dt*(omega_z(k-1) + b_gC(k-1));   % Euler Integration

    Cb2n = [cos(psiC(k-1)) -sin(psiC(k-1));sin(psiC(k-1)) cos(psiC(k-1))];

    aC = Cb2n*[f_x(k-1) + b_axC(k-1);f_y(k-1) + b_ayC(k-1)];            %   Computed Acceleration
    
    v_NC(k) = v_NC(k-1) + dt*aC(1);
    v_EC(k) = v_EC(k-1) + dt*aC(2);
    
    p_NC(k) = p_NC(k-1) + dt*v_NC(k-1);
    p_EC(k) = p_EC(k-1) + dt*v_EC(k-1);
    
    b_axC(k) = b_axC(k-1);
    b_ayC(k) = b_ayC(k-1);
    b_gC(k) = b_gC(k-1);
    
%   Measurement Update

    if (num_of_tu >= tu_per_mu && FILTER_FLAG == 1)
        
        num_of_tu = 0;
        
        K = P*H'*inv(H*P*H' + R);
        P = (eye(8) - K*H)*P;
    
        x_KF = [p_NC(k);p_EC(k);v_NC(k);v_EC(k);psiC(k);b_axC(k);b_ayC(k);b_gC(k)];
        
        if(strcmp(FILTER_ARCHITECTURE,'PVH'))
            z_GNSS = [p_N_GNSS(k);p_E_GNSS(k);v_N_GNSS(k);v_E_GNSS(k);psi_compass(k)];
        elseif(strcmp(FILTER_ARCHITECTURE,'PV'))
            z_GNSS = [p_N_GNSS(k);p_E_GNSS(k);v_N_GNSS(k);v_E_GNSS(k)];
        elseif(strcmp(FILTER_ARCHITECTURE,'PH'))
            z_GNSS = [p_N_GNSS(k);p_E_GNSS(k);psi_compass(k)];
        elseif(strcmp(FILTER_ARCHITECTURE,'P'))
            z_GNSS = [p_N_GNSS(k);p_E_GNSS(k)];
        end

        x_KF = x_KF + K*(z_GNSS - H*x_KF);
    
        p_NC(k) = x_KF(1);
        p_EC(k) = x_KF(2);
        
        v_NC(k) = x_KF(3);
        v_EC(k) = x_KF(4);
        
        psiC(k) = x_KF(5);
        
        b_axC(k) = x_KF(6);
        b_ayC(k) = x_KF(7);
        b_gC(k) = x_KF(8);
        
    end

    num_of_tu = num_of_tu + 1;
    waitbar(k/drl);    
    
end

%==================================================================================%
% (f)                                Save Data                                     %
%==================================================================================%

if (SAVE_FLAG)
    fileName = ['TRAJECTORY_',TRAJECTORY,'_FILTER_ARCHITECTURE_',FILTER_ARCHITECTURE,'.mat'];
    if(ispc)
        eval(['save -binary .\New_Data\',fileName,' ','t p_NC p_EC v_NC v_EC psiC b_axC b_ayC b_gC;']);
    else
        eval(['save -binary ./New_Data/',fileName,' ','t p_NC p_EC v_NC v_EC psiC b_axC b_ayC b_gC;']);
    end
end