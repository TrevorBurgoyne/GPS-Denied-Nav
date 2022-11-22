%% IMU Initialization Trajectory
% GPS Denied Navigation, AEM 4331 Fall 2022

function [pos, orient, vel, acc, angvel, times, wps, t] = init_trajectory(Fs)
    % waypointTrajectory for a kinda random 20 min test trajectory.
    %   USAGE: `[position,orientation,velocity,acceleration,angularVelocity,wps,t] = trajectory1(Fs);` 
    %   will initialize a waypointTrajectory object with the pre-defined waypoints in this
    %   file, with Fs being the sample rate (Hz). wps is a list of the
    %   waypoints and t contains the corresponding times when those
    %   waypoints are reached. `pos, orient, vel, acc, angvel, times`
    %   will have the full trajectory details at each `times` entry.

    % Waypoints (m)
    wps = [0 0 0
           0 0 0];
    
    % Times (s)
    t  = [0
          60];
    
    traj = waypointTrajectory(wps, t, 'SampleRate', Fs);
    % traj = waypointTrajectory(wps, t, 'SampleRate', Fs, 'AutoBank', true, 'AutoPitch', true);
    times = t(1):1/Fs:t(end);
    [pos, orient, vel, acc, angvel] = lookupPose(traj, times);
end