%% LEO Satellite Model
% GPS Denied Navigation, AEM 4331 Fall 2022
% Get a position and velocity estimate using LEO satellite signals.
% https://celestrak.org/

% Methodoology: LEO satellites broadcast a packet that includes the time of
% broadcast and the satellite name/id. Upon reception by the sensor, the
% satellite's orbital elements are drawn from an onboard database, and
% using the time of broadcast, the satellite's position at the time of
% broadcast is estimated. With four or more such signals, the distance to
% each satellite as well as the time onboard the sensor can be calculated
% using basic principles of triangulation.

function leo = leo_model(sample_rate)
    % Constructor
    if nargin > 0
        Fs = sample_rate;
    else
        Fs = 1; % 1 Hz
    end
    
    % TODO: everything :P

end