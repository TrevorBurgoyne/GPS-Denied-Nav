% ADOPDemo.m
%
% This script gives some examples of how to work with the ambiguity DOP functions provided
% with the textbook.
%

%% Constants

% speed of light
c = 299799458;

% GPS L1 frequency
fL1 = 1575.42e6;

% GPS L1 Wavelength
wL1 = c / fL1;


%% Define the Satellite Geometry

% NOTE: This could be updated using information from mission planning software

% assume eight satellites spaced equally around the horizon plus one at the zenith (total of nine)
satAz = [  0 , 45:45:360 ];
satEl = [ 90 , zeros( 1, length( satAz ) - 1 ) ];


%% Effect of Pseudorange Accuracy on ADOP

% define the undifferenced pseudorange measurement standard deviations (in cycles)
cpStdDev = 0.05;

% define the undifferenced pseudorange measurement standard deviations (in metres)
prStdDev = 0.1:0.1:5;

% define the INS position covariance matrix per axis (in metres^2)
insPosStdDev = 0.5;

% compute the ADOP values
for i = 1:length( prStdDev ) 
   
   % compute the GNSS/INS ambiguity covariance matrix
   pnnGnss = GnssOnlyPNN( satAz, satEl, prStdDev(i)^2, cpStdDev^2, wL1 );
   
   % compute the GNSS/INS ambiguity covariance matrix
   pnnGnssIns = GnssInsPNN( satAz, satEl, prStdDev(i)^2, cpStdDev^2, insPosStdDev^2 * eye(3), wL1 );
   
   % compute the ADOP
   adopGnss(i) = ADOP( pnnGnss );
   adopGnssIns(i) = ADOP( pnnGnssIns );
   
end

figure
plot( prStdDev, adopGnss, prStdDev, adopGnssIns );
xlabel( '\bfPseudorange Standard Deviation (m)' );
ylabel( '\bfADOP (proportional to volume of ambiguity search space)' );
title( sprintf( '\\bfEffect of Pseudorange Accuracy on ADOP (INS Std Dev: %.1f m per axis)', insPosStdDev ) );
legend( 'GNSS-Only', 'GNSS/INS', 'Location', 'NorthWest' );


%% Effect of INS Position Accuracy on ADOP

% define the undifferenced pseudorange measurement standard deviations (in cycles)
cpStdDev = 0.05;

% define the undifferenced pseudorange measurement standard deviations (in metres)
prStdDev = 1;

% define the INS position covariance matrix per axis (in metres^2)
insPosStdDev = 0.1:0.1:5;

% clear previous answers
clear adopGnss;
clear adopGnssIns;

% compute the ADOP values
for i = 1:length( insPosStdDev ) 
   
   % compute the GNSS/INS ambiguity covariance matrix
   pnnGnss = GnssOnlyPNN( satAz, satEl, prStdDev^2, cpStdDev^2, wL1 );
   
   % compute the GNSS/INS ambiguity covariance matrix
   pnnGnssIns = GnssInsPNN( satAz, satEl, prStdDev^2, cpStdDev^2, insPosStdDev(i)^2 * eye(3), wL1 );
   
   % compute the ADOP
   adopGnss(i) = ADOP( pnnGnss );
   adopGnssIns(i) = ADOP( pnnGnssIns );
   
end

figure
plot( insPosStdDev, adopGnss, insPosStdDev, adopGnssIns );
xlabel( '\bfINS Position Standard Deviation per Axis (m)' );
ylabel( '\bfADOP (proportional to volume of ambiguity search space)' );
title( sprintf( '\\bfEffect of INS Position Accuracy on ADOP (PR Std Dev: %.1f m)', prStdDev ) );
legend( 'GNSS-Only', 'GNSS/INS', 'Location', 'NorthWest' );


