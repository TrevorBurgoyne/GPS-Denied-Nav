% %===========================================================%
% %                   wgs_84_parameters.m                     %
% %                                                           %
% %   This function returns the paramters describing the      %
% %   WGS-84 reference ellipsoid.                             %
% %                                                           %
% %   Programmer:     Demoz Gebre-Egziabher                   %
% %   Created:        July 2, 1998                            %
% %   Last Modified:  March 26, 2009                          %
% %                                                           %
% %===========================================================%

f = 1/298.257223563;        %   WGS-84 Flattening.
e = sqrt(f*(2 - f));        %   Eccentricity.
omega_ie = 7.292115e-5;     %   WGS-84 Earth rate (rad/s).
R_0 = 6378137;              %   WGS-84 equatorial radius (m).                            
R_P = R_0*(1 - f);          %   Polar radius (m).
mu_E = 3.986004418e14;      %   WGS-84 Earth's gravitational

%===========================================================%     
