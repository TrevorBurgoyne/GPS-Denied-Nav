%% Test Trajectory 1
% GPS Denied Navigation, AEM 4331 Fall 2022

function [pos, orient, vel, acc, angvel, times, wps, t] = trajectory1(Fs)
    % waypointTrajectory for a straight (no y motion), 20 min test trajectory.
    %   USAGE: `[position,orientation,velocity,acceleration,angularVelocity,wps,t] = trajectory1(Fs);` 
    %   will initialize a waypointTrajectory object with the pre-defined waypoints in this
    %   file, with Fs being the sample rate (Hz). wps is a list of the
    %   waypoints and t contains the corresponding times when those
    %   waypoints are reached. `pos, orient, vel, acc, angvel, times`
    %   will have the full trajectory details at each `times` entry.

    [wps1, t1, x0, y0, z0, t0] = launch_trajectory(); % start with the launch
    tf = 20*60;        % s, 20 min flight
    v  = 100;          % m/s, steady flight speed
    xf = x0+v*(tf-t0); % m, final x position

    % Waypoints (m)
    wps2 = [xf, y0, z0];
    wps = [wps1', wps2']'; % append to launch
    
    % Times (s)
    t2  = [tf];
    t = [t1', t2']'; % append to launch
    
    traj = waypointTrajectory(wps, t, 'SampleRate', Fs);
    times = t(1):1/Fs:t(end);
    [pos, orient, vel, acc, angvel] = lookupPose(traj, times);
end