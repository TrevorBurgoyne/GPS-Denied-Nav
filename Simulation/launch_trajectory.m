%% Launch Trajectory
% GPS Denied Navigation, AEM 4331 Fall 2022

function [wps, t] = launch_trajectory()

    % Launch from a 15° slope at <30Gs to an initial velocity of 70 m/s
    angle = 15; % deg
    a = 25*conversions.g; % 25Gs
    v0 = 70; % m/s
    t0 = v0/a; % s, time to accelerate to the initial velocity
    d = .5*a*t0^2; % m, total distance traveled during launch
    x = d*cosd(angle); % m, total x distance traveled
    z = d*sind(angle); % m, total z distance traveled

    wps = [0 0 0;  % 1
           x 0 z]; % 2
           
    t   = [0       % 1
           t0];    % 2
end