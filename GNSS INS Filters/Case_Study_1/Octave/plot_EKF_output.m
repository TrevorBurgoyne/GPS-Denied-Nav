%===========================================================%
%                       plot_EKF_output.m                   %
%                                                           %
%   This m-script plots the figures for Case Study 1 shown  %
%   shown.                                                  %
%                                                           %
%   Programmer:     Demoz Gebre-Egziabher                   %
%   Created:        March 26, 2009                          %
%   Last Modified:  June 26, 2009                           %
%                                                           %
%===========================================================%


close all;

%   Figure 1:  Ground Track

start_lon = gps_pos_lla(1,2)*ones(drl,1);
start_lat = gps_pos_lla(1,1)*ones(drl,1);

lla_east = (gps_pos_lla(:,2)-start_lon);
lla_north = (gps_pos_lla(:,1)-start_lat);

figure(gcf)
h1 = plot((R_0/cos(start_lat(1)))*lla_east,R_0*lla_north,'r-');hold on;grid on;axis equal;
title('Vehicle Trajectory'); xlabel('East/West (m)');ylabel('North/South (m)');
set(h1,'LineWidth',2);
text((R_0/cos(start_lat(1)))*lla_east(1),R_0*lla_north(1),'Start');
text((R_0/cos(start_lat(1)))*lla_east(end)+1,R_0*lla_north(end),'Stop');

%   Figure 2:   Attitude Estimates

figure(gcf+1)
subplot(311)
plot(t/60,yaw,'r-',t/60,eul_ins(:,3)*r2d,'b-','linewidth',2);
legend('MIDG II Solution','GNSS/INS solution');
title('Yaw angle');
xlabel('time (min)');
ylabel('\psi (deg)');
grid on;
    
subplot(312)
plot(t/60,pitch,'r-',t/60,eul_ins(:,2)*r2d,'b-','linewidth',2);
title('Pitch angle');
xlabel('time (min)');
ylabel('\theta (deg)');
grid on;

subplot(313)
plot(t/60,roll,'r-',t/60,eul_ins(:,1)*r2d,'b-','linewidth',2);
title('Roll angle');
xlabel('time (min)');
ylabel('\phi (deg)');
grid on;

%   Figure 3:   Sensor Bias Estimates

figure(gcf+1)
subplot(321)
h1 = plot(t/60,accelBias(:,1)*(1000/g),'b');hold on;grid on;%estimate
title('Accelerometer Bias Estimates');
ylabel('b_{a_1}(milli-g)');
%axis([t(1)/60 14 -150 150]);
set(h1,'LineWidth',2);

subplot(323)
h2 = plot(t/60,accelBias(:,2)*(1000/g),'b');hold on;grid on;
ylabel('b_{a_2} (milli-g)');
%axis([t(1)/60 14 -150 150]);
set(h2,'LineWidth',2);

subplot(325)
h3 = plot(t/60,accelBias(:,3)*(1000/g),'b');hold on;grid on;
ylabel('b_{a_3} Bias(milli-g)');
xlabel('Time (min)');
%axis([t(1)/60 14 -150 150]);
set(h3,'LineWidth',2);

subplot(322)
h4 = plot(t/60,gyroBias(:,1)*r2d,'b');hold on;grid on;
title('Gyro Bias Estimates');
ylabel('b_{g_1} (deg/s)');
set(h4,'LineWidth',2);
%axis([t(1)/60 14 -4 4]);
subplot(324)
h5 = plot(t/60,gyroBias(:,2)*r2d,'b');hold on;grid on;
ylabel('b_{g_2} (deg/s)');
set(h5,'LineWidth',2);
%axis([t(1)/60 t(end)/60 -4 4]);

subplot(326)
h6 = plot(t/60,gyroBias(:,3)*r2d,'b');hold on;grid on;
ylabel('b_{g_3} (deg/s)');
xlabel('Time (min)');
set(h6,'LineWidth',2);
%axis([t(1)/60 14 -4 4]);