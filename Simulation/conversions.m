%% Unit conversions
% GPS Denied Navigation, AEM 4331 Fall 2022

classdef conversions
    methods (Static)
        function ret = deg_per_hr_to_rad_per_sec(val)
            % deg/hr -> rad/s
            ret = (val*pi)/(180*3600);
        end
        
        function ret = deg_per_sqrt_hr_to_rad_per_sqrt_sec(val)
            % deg/sqrt(hr) -> rad/sqrt(s) = (rad/s)/sqrt(Hz)
            ret = (val*pi)/(180*60); 
        end
    end
    
end