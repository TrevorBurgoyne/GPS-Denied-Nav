%% Unit conversions
% GPS Denied Navigation, AEM 4331 Fall 2022
classdef conversions
    properties (Constant)
        g  = 9.81;   % m/s^2
    end
    
    methods (Static)
        function ret = deg_per_hr_to_rad_per_sec(val)
            % deg/hr -> rad/s
            ret = (val*pi)/(180*3600);
        end
        
        function ret = deg_per_sqrt_hr_to_rad_per_sqrt_sec(val)
            % deg/sqrt(hr) -> rad/sqrt(s) = (rad/s)/sqrt(Hz)
            ret = (val*pi)/(180*60); 
        end
        
        function ret = mg_to_m_per_s_squared(val)
            % mg -> m/s^2
            ret = (val/1000)*conversions.g;
        end
        
        function ret = fps_per_sqrt_hr_to_m_per_sec_per_sqrt_sec(val)
            % fps/sqrt(hr) -> (m/s)/sqrt(s) = m/s^2/sqrt(Hz)
            ret = (val*0.3048)/(60);
        end
    end
    
end