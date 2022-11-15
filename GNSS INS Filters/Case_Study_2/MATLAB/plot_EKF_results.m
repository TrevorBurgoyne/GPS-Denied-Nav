            function plot_EKF_results(TRAJECTORY,FILTER_ARCHITECTURE)
%==================================================================================%
%           function plot_ekf_outputs(TRAJECTORY,FILTER_ARCHITECTURE)
%
%   This file plots the results from the 2-D aided INS simulatioin.
%
%  Programmer:      Demoz Gebre-Egziabher
%  Created:         February 16, 2007
%  Last Modified:   Feb. 11, 2007
%
%==================================================================================%

close all;clc;

%==================================================================================%
% (a)                         Input Argument Check                                 %
%==================================================================================%

if (nargin < 2)
    error('A Motion Profile and/or a Filter Architecture Must be Selected.');
end

%==================================================================================%
% (b)                      Define Conversion Constants                             %
%==================================================================================%

d2r = pi/180;
r2d = 1/d2r;
g = 9.81;

%==================================================================================%
% (b)                         Load in Appropriate Files                            %
%==================================================================================%

if(ispc)
    pathName = '.\New_Data\';
else
    pathName = './New_Data/';
end

fileName_input = [pathName,'TRAJECTORY_',TRAJECTORY,'.mat'];
fileName_output = [pathName,'TRAJECTORY_',TRAJECTORY,'_FILTER_ARCHITECTURE_',FILTER_ARCHITECTURE,'.mat'];

eval(['load ',fileName_input]);
eval(['load ',fileName_output]);

%==================================================================================%
% (d)                         Process the Loaded Data                              %
%==================================================================================%

drl = length(t);

%==================================================================================%
% (e)                         Begin Plotting the Data                              %
%==================================================================================%
% 
% figure(gcf)
% subplot(211)
% plot(y,r2d*psiT,'r-',y,r2d*psiC,'g--',y,r2d*psiGPS,'b.');
% legend('true','Inertial','GPS');grid on;ylabel('\Psi');
% subplot(212)
% plot(y,r2d*(psiT-psiGPS),'g-',y,r2d*(psiT-psiC),'b--');grid on;
% legend('Truth-GPS','Truth-Inertial');
% ylabel('\delta \Psi');xlabel('y-Distance');

% figure(gcf)
% subplot(311)
% plot(p_E,p_N,'r-',p_EC,p_NC,'g--',p_E_GNSS,p_N_GNSS,'b.');grid on;ylabel('North Position (m)');
% legend('Truth','EKF Estimate','GNSS');grid on;
% subplot(312)
% plot(y,y-yGPS,'b--',y,y-yC,'g-');grid on;ylabel('\delta y');
% legend('Truth-Inertial','Truth-Inertial');
% subplot(313)
% plot(y,x-xC,'b--',y,x-xC,'g-');grid on;ylabel('\delta x');
% legend('Truth-Inertial','Truth-Inertial');
% xlabel('Y-Distance');

% figure(gcf)
figure()
plot(p_E,p_N,'r-',p_EC,p_NC,'g--',p_E_GNSS,p_N_GNSS,'b.');grid on;ylabel('North Position (m)');
legend('Truth','EKF Estimate','GNSS');grid on;axis('equal');
xlabel('East Position (m)');
title(['Ground Track. TRAJECTORY = ',TRAJECTORY,'.  FILTER ARCHITECTURE = ',FILTER_ARCHITECTURE,'.']);

% figure(gcf+1)
figure()
subplot(311)
plot(t,p_N,'b-', t,p_NC,'r--',t,p_N_GNSS,'g-');grid on;ylabel('p_N (m)');
legend('Truth','EKF','GNSS');
subplot(312)
plot(t,p_E,'b-',t,p_EC,'r--',t,p_E_GNSS,'g-');grid on;ylabel('p_E(m)');
legend('Truth','EKF','GNSS');
subplot(313)
plot(t,r2d*(psi_true),'b-',t,r2d*(psiC),'r--',t,r2d*(psi_compass),'g-');grid on;ylabel(' \psi_{nb} (deg)');
legend('Truth','EKF','Compass');
xlabel('Time (sec)');
title(['Position/Heading History. TRAJECTORY = ',TRAJECTORY,'.  FILTER ARCHITECTURE = ',FILTER_ARCHITECTURE,'.']);

% figure(gcf+1)
figure()
subplot(311)
plot(t,p_N-p_NC,'b--',t,p_N-p_N_GNSS,'g-');grid on;ylabel('\delta p_N (m)');
legend('EKF-Truth','GNSS-Truth');
subplot(312)
plot(t,p_E-p_EC,'b--',t,p_E-p_E_GNSS,'g-');grid on;ylabel('\delta p_E (m)');
legend('EKF-Truth','GNSS-Truth');
subplot(313)
plot(t,r2d*(psiC-psi_true),'b--',t,r2d*(psi_compass-psi_true),'g-');grid on;ylabel('\delta \psi_{nb} (deg)');
legend('EKF-Truth','Compass-Truth');
xlabel('Time (sec)');
title(['Position/Heading Error History. TRAJECTORY = ',TRAJECTORY,'.  FILTER ARCHITECTURE = ',FILTER_ARCHITECTURE,'.']);

% figure(gcf+1)
figure()
subplot(211)
plot(t,v_N,'b-',t,v_NC,'r--',t,v_N_GNSS,'g-');grid on;ylabel('v_N (m/s)');
legend('Truth','EKF','GNSS');
subplot(212)
plot(t,v_E,'b-',t,v_EC,'r--',t,v_E_GNSS,'g-');grid on;ylabel('v_E (m/s)');
legend('Truth','EKF','GNSS');
xlabel('Time (sec)');
title(['Velocity History. TRAJECTORY = ',TRAJECTORY,'.  FILTER ARCHITECTURE = ',FILTER_ARCHITECTURE,'.']);

% figure(gcf+1)
figure()
subplot(211)
plot(t,v_N-v_NC,'b--',t,v_N-v_N_GNSS,'g-');grid on;ylabel('\delta v_N (m/s)');
legend('EKF-Truth','GNSS-Truth');
subplot(212)
plot(t,v_E-v_EC,'b--',t,v_E-v_E_GNSS,'g-');grid on;ylabel('\delta v_E (m/s)');
legend('EKF-Truth','GNSS-Truth');
xlabel('Time (sec)');
title(['Velocity Error History. TRAJECTORY = ',TRAJECTORY,'.  FILTER ARCHITECTURE = ',FILTER_ARCHITECTURE,'.']);

% figure(gcf+1)
figure()
subplot(311)
plot(t,-b_ax/g,'b',t,b_axC/g,'r--');grid on;ylabel('b_{ax} (g)');
title(['Sensor Bias Estimates. TRAJECTORY = ',TRAJECTORY,'.  FILTER ARCHITECTURE = ',FILTER_ARCHITECTURE,'.']);
subplot(312)
plot(t,-b_ay/g,'b',t,b_ayC/g,'r--');grid on;ylabel('b_{ay} (g)');
subplot(313)
plot(t,-r2d*b_g,'b',t,r2d*b_gC,'r--');grid on;ylabel('b_{g} (deg/sec)');
xlabel('Time (sec)');


