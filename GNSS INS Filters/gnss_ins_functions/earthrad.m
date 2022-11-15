            function [R_N, R_E] = earthrad(L)
%===========================================================%
%           function [R_N, R_E] = earthrad(L)               %
%                                                           %
%   This functions computes the north-south and east-west   %
%   radii of curvature (R_N and R_E, respectively) for      %
%   the WGS-84 as a function of geodetic latitude (L).      %
%   The input, L, is in units of radians and the outputs    %
%   R_N and R_E are in units of meters.                     %
%                                                           %
%   Programmer:     Demoz Gebre-Egziabher                   %
%   Created:        July 2, 1998                            %
%   Last Modified:  March 26, 2009                          %
%                                                           %
%===========================================================%     

%   Load ellipsoid constants

wgs_84_parameters;

%   Compute radii

k = sqrt(1 - (e*sin(L))^2);
R_N = R_0*(1 - e^2)/k^3;
R_E = R_0/k;

%===========================================================%
