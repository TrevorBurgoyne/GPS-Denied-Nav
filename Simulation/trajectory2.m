%% Test Trajectory 2
% GPS Denied Navigation, AEM 4331 Fall 2022

function [pos, orient, vel, acc, angvel, times, wps, t] = trajectory2(Fs)
    % waypointTrajectory for a kinda random 20 min test trajectory.
    %   USAGE: `[position,orientation,velocity,acceleration,angularVelocity,wps,t] = trajectory1(Fs);` 
    %   will initialize a waypointTrajectory object with the pre-defined waypoints in this
    %   file, with Fs being the sample rate (Hz). wps is a list of the
    %   waypoints and t contains the corresponding times when those
    %   waypoints are reached. `pos, orient, vel, acc, angvel, times`
    %   will have the full trajectory details at each `times` entry.

    [wps1, t1, x0, y0, z0, t0] = launch_trajectory(); % start with the launch
    tf = 20*60;        % s, end time for a 20 min flight

    % Waypoints (m)
    wps2 = [x0+4e3,  y0-1e3,  z0+7e3;  % 1
            x0+5e3,  y0+2e3,  z0+9e3;  % 2
            x0+8e3,  y0+4e3,  z0+4e3;  % 3
            x0+14e3, y0-1e3,  z0+9e3;  % 4
            x0+18e3, y0-5e3,  z0+7e3;  % 5
            x0+25e3, y0-7e3,  z0+10e3; % 6
            x0+33e3, y0-9e3,  z0+14e3; % 7
            x0+38e3, y0-11e3, z0+14e3; % 8
            x0+45e3, y0-7e3,  z0+7e3;  % 9
            x0+46e3, y0-6e3,  z0+20e3; % 10
            x0+55e3, y0-4e3,  z0+23e3; % 11
            x0+59e3, y0-1e3,  z0+18e3; % 12
            x0+69e3, y0+5e3,  z0+18e3; % 13
            x0+61e3, y0+10e3, z0+18e3; % 14
            x0+69e3, y0+11e3, z0+25e3; % 15
            x0+74e3, y0+15e3, z0+28e3; % 16
            x0+80e3, y0+14e3, z0+27e3; % 17
            x0+83e3, y0+16e3, z0+28e3; % 18
            x0+88e3, y0+16e3, z0+30e3; % 19
            x0+91e3, y0+18e3, z0+33e3; % 20
    ];
    wps = [wps1', wps2']'; % append to launch
    
    % Times (s)
    t2  = [t0 +  1*60; % 1
           t0 +  2*60; % 2
           t0 +  3*60; % 3
           t0 +  4*60; % 4
           t0 +  5*60; % 5
           t0 +  6*60; % 6
           t0 +  7*60; % 7
           t0 +  8*60; % 8
           t0 +  9*60; % 9
           t0 + 10*60; % 10
           t0 + 11*60; % 11
           t0 + 12*60; % 12
           t0 + 13*60; % 13
           t0 + 14*60; % 14
           t0 + 15*60; % 15
           t0 + 16*60; % 16
           t0 + 17*60; % 17
           t0 + 18*60; % 18
           t0 + 19*60; % 19    
           tf];        % 20
    t = [t1', t2']'; % append to launch
    
    traj = waypointTrajectory(wps, t, 'SampleRate', Fs);
    % traj = waypointTrajectory(wps, t, 'SampleRate', Fs, 'AutoBank', true, 'AutoPitch', true);
    times = t(1):1/Fs:t(end);
    [pos, orient, vel, acc, angvel] = lookupPose(traj, times);
end