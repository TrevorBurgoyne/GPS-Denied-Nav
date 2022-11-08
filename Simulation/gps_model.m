%% GPS Model
% GPS Denied Navigation, AEM 4331 Fall 2022
% Used for comparison with other solutions

function gps = gps_model(sample_rate)
    % Constructor
    if nargin > 0
        Fs = sample_rate;
    else
        Fs = 1; % 1 Hz
    end

    % LLA position for Natick, MA
    refLoc = [42.2825 -71.343 53.0352];

    gps = gpsSensor(...
      'SampleRate', Fs, ...
      'ReferenceLocation', refLoc ...
    ); % By default, uses local cartesian coords as input and outputs LLA
end