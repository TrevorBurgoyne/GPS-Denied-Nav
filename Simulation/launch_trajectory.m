%% Launch Trajectory
% GPS Denied Navigation, AEM 4331 Fall 2022

function [wps, t, x0, y0, z0, t0] = launch_trajectory()
    % Details of a generic launch routine.
        %   USAGE: `[wps, t, x0, z0, t0] = launch_trajectory();` 
        %   wps: launch waypoints
        %   t:   time at each waypoint
        %   x0:  final x position after launch
        %   z0:  final z position after launch
        %   t0:  time at end of launch

    % Launch from a 15° slope at <30Gs to an initial velocity of 70 m/s
    angle = 15; % deg
    a = 25*conversions.g; % 25Gs
    v = 70; % m/s
    ta = v/a; % s, time to accelerate to the initial velocity
    d = .5*a*ta^2; % m, total distance traveled during launch
    x = d*cosd(angle); % m, total x distance traveled
    z = d*sind(angle); % m, total z distance traveled

    wps = [0 0 0;  % 1
           x 0 z]; % 2
           
    t   = [0       % 1
           ta];    % 2

    x0 = wps(end,1);  % m, final x position
    y0 = wps(end,2);  % m, final y position
    z0 = wps(end,3);  % m, final z position
    t0 = t(end);      % s, end of launch phase
end