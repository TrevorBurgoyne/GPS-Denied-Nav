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
% load '../../../Simulation/Trajectory2.mat'
% load './traj2_ekf_output.mat'

% True x/y vs GPS x/y vs Solution x/y
[East,North,Up] = latlon2local(rad2deg(pos_ins(:,1)), rad2deg(pos_ins(:,2)), pos_ins(:,3), gps.ReferenceLocation);
pos_ins_local = [North, East, -Up];

% figure()
% hold on;grid on;axis equal;
% title('Vehicle Trajectory'); xlabel('x(m)');ylabel('y(m)');
% plot(pos_ins_local(:,1),pos_ins_local(:,2),'r-', 'DisplayName', 'INS');
% % plot(gps_pos_local(:,1),gps_pos_local(:,2),'green', 'DisplayName', 'GPS');
% plot(truth_pos(:,1), truth_pos(:,2),'blue','DisplayName','Truth');
% legend()


%% 3D True Trajectory
figure()
plot3(truth_pos(:,1), truth_pos(:,2), truth_pos(:,3))
grid on
title('3D Trajectory')
xlabel('x(m)');ylabel('y(m)');zlabel('z(m)')

%% Seperate x,y,z plots
figure()
n_row = 3;
n_col = 2;
plot_n = 1;

% Error
err = pos_ins_local - truth_pos;

% x
subplot(n_row, n_col, plot_n)
plot_n = plot_n + 1;
hold on
plot(t, err(:,1), 'DisplayName','Error')
title('x Error')
xlabel('s')
ylabel('m')
legend('show','Location','best');

subplot(n_row, n_col, plot_n)
plot_n = plot_n + 1;
hold on
plot(t, truth_pos(:,1), 'DisplayName','Truth')
plot(t, pos_ins_local(:,1), 'DisplayName','GNSS/INS')
title('x Position')
xlabel('s')
ylabel('m')
legend('show','Location','best');

% y
subplot(n_row, n_col, plot_n)
plot_n = plot_n + 1;
hold on
plot(t, err(:,2), 'DisplayName','Error')
title('y Error')
xlabel('s')
ylabel('m')
legend('show','Location','best');

subplot(n_row, n_col, plot_n)
plot_n = plot_n + 1;
hold on
plot(t, truth_pos(:,2), 'DisplayName','Truth')
plot(t, pos_ins_local(:,2), 'DisplayName','GNSS/INS')
title('y Position')
xlabel('s')
ylabel('m')
legend('show','Location','best');

% z
subplot(n_row, n_col, plot_n)
plot_n = plot_n + 1;
hold on
plot(t, err(:,3), 'DisplayName','Error')
title('z Error')
xlabel('s')
ylabel('m')
legend('show','Location','best');

subplot(n_row, n_col, plot_n)
plot_n = plot_n + 1;
hold on
plot(t, truth_pos(:,3), 'DisplayName','Truth')
plot(t, pos_ins_local(:,3), 'DisplayName','GNSS/INS')
title('z Position')
xlabel('s')
ylabel('m')
legend('show','Location','best');

%   Figure 1:  Ground Track

% % start_lon = gps_pos_lla(1,2)*ones(drl,1);
% % start_lat = gps_pos_lla(1,1)*ones(drl,1);
% start_lon = truth_pos_lla(1,2)*ones(drl,1);
% start_lat = truth_pos_lla(1,1)*ones(drl,1);
% 
% lla_east = (gps_pos_lla(:,2)-start_lon);
% lla_north = (gps_pos_lla(:,1)-start_lat);
% 
% truth_lla_east = (truth_pos_lla(:,2)-start_lon);
% truth_lla_north = (truth_pos_lla(:,1)-start_lat);
% 
% % figure(gcf)
% figure()
% h1 = plot((R_0/cos(start_lat(1)))*lla_east,R_0*lla_north,'r-', 'DisplayName', 'GNSS/INS');
% hold on;grid on;axis equal;
% plot((R_0/cos(start_lat(1)))*truth_lla_east,R_0*truth_lla_north,'blue','DisplayName', 'Truth')
% title('Vehicle Trajectory'); xlabel('East/West (m)');ylabel('North/South (m)');
% set(h1,'LineWidth',2);
% text((R_0/cos(start_lat(1)))*lla_east(1),R_0*lla_north(1),'Start');
% text((R_0/cos(start_lat(1)))*lla_east(end)+1,R_0*lla_north(end),'Stop');
% legend()

%%   Figure 2:   Attitude Estimates

hold off
% figure(gcf+1)
figure()
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

hold off
%% Velocity and Acceleration Plots
% Error
err = vel_ins - vel;

figure
n_row = 3;
n_col = 2;
plot_n = 1;

% x
subplot(n_row, n_col, plot_n)
plot_n = plot_n + 1;
plot(t/60, err(:,1), 'DisplayName', 'Error')
title('x Velocity Error')
xlabel('Time (min)')
ylabel('Velocity (m/s)')
grid on
grid minor

subplot(n_row, n_col, plot_n)
plot_n = plot_n + 1;
plot(t/60, vel_ins(:,1), 'DisplayName', 'GNSS/INS')
plot(t/60, vel(:,1), 'DisplayName', 'Truth')
title('x Velocity')
xlabel('Time (min)')
ylabel('Velocity (m/s)')
grid on
grid minor

% y
subplot(n_row, n_col, plot_n)
plot_n = plot_n + 1;
plot(t/60, err(:,2), 'DisplayName', 'Error')
title('y Velocity Error')
xlabel('Time (min)')
ylabel('Velocity (m/s)')
grid on
grid minor

subplot(n_row, n_col, plot_n)
plot_n = plot_n + 1;
plot(t/60, vel_ins(:,2), 'DisplayName', 'GNSS/INS')
plot(t/60, vel(:,2), 'DisplayName', 'Truth')
title('y Velocity')
xlabel('Time (min)')
ylabel('Velocity (m/s)')
grid on
grid minor

% z
subplot(n_row, n_col, plot_n)
plot_n = plot_n + 1;
plot(t/60, err(:,3), 'DisplayName', 'Error')
title('z Velocity Error')
xlabel('Time (min)')
ylabel('Velocity (m/s)')
grid on
grid minor

subplot(n_row, n_col, plot_n)
plot_n = plot_n + 1;
plot(t/60, vel_ins(:,3), 'DisplayName', 'GNSS/INS')
plot(t/60, vel(:,3), 'DisplayName', 'Truth')
title('z Velocity')
xlabel('Time (min)')
ylabel('Velocity (m/s)')
grid on
grid minor

hold off

% Acceleration Plots

figure
subplot(311)
plot(t/60, accelBias(:,1), 'DisplayName', 'North')
title('North Acceleration')
xlabel('Time (min)')
ylabel('Acceleration (m/s/s)')
grid on
grid minor

subplot(312)
plot(t/60, accelBias(:,2), 'DisplayName', 'East')
title('East Acceleration')
xlabel('Time (min)')
ylabel('Acceleration (m/s/s)')
grid on
grid minor

subplot(313)
plot(t/60, accelBias(:,3), 'DisplayName', 'Down')
title('Down Acceleration')
xlabel('Time (min)')
ylabel('Acceleration (m/s/s)')
grid on
grid minor

hold off
%%   Figure 3:   Sensor Bias Estimates

% figure(gcf+1)
% figure()
% subplot(321)
% h1 = plot(t/60,accelBias(:,1)*(1000/g),'b');hold on;grid on;%estimate
% title('Accelerometer Bias Estimates');
% ylabel('b_{a_1}(milli-g)');
% %axis([t(1)/60 14 -150 150]);
% set(h1,'LineWidth',2);
% 
% subplot(323)
% h2 = plot(t/60,accelBias(:,2)*(1000/g),'b');hold on;grid on;
% ylabel('b_{a_2} (milli-g)');
% %axis([t(1)/60 14 -150 150]);
% set(h2,'LineWidth',2);
% 
% subplot(325)
% h3 = plot(t/60,accelBias(:,3)*(1000/g),'b');hold on;grid on;
% ylabel('b_{a_3} Bias(milli-g)');
% xlabel('Time (min)');
% %axis([t(1)/60 14 -150 150]);
% set(h3,'LineWidth',2);
% 
% subplot(322)
% h4 = plot(t/60,gyroBias(:,1)*r2d,'b');hold on;grid on;
% title('Gyro Bias Estimates');
% ylabel('b_{g_1} (deg/s)');
% set(h4,'LineWidth',2);
% %axis([t(1)/60 14 -4 4]);
% subplot(324)
% h5 = plot(t/60,gyroBias(:,2)*r2d,'b');hold on;grid on;
% ylabel('b_{g_2} (deg/s)');
% set(h5,'LineWidth',2);
% %axis([t(1)/60 t(end)/60 -4 4]);
% 
% subplot(326)
% h6 = plot(t/60,gyroBias(:,3)*r2d,'b');hold on;grid on;
% ylabel('b_{g_3} (deg/s)');
% xlabel('Time (min)');
% set(h6,'LineWidth',2);
%axis([t(1)/60 14 -4 4]);