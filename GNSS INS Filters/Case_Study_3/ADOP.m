function adop = ADOP( PNN )
%
% adop = ADOP( PNN )
%
% SUMMARY: 
%    Computes the Ambiguity Dilution of Precision (ADOP) value based on a given ambiguity covariance
%    matrix.
%
% INPUT:
%  - PNN: The ambiguity covariance matrix for which the ADOP is to be computed.
%
% OUTPUT:
%  - adop: The ADOP of the of the given ambiguity covariance matrix.
%
% COPYRIGHT:
%    (c) 2009
%    Dr. Mark G. Petovello
%    Position, Location And Navigation (PLAN) Group
%    Department of Geomatics Engineering
%    University of Calgary
%    2500 University Drive N.W.
%    Calgary, AB
%    T2N 1N4
%    CANADA
%

%% Data Checking

% check for square
if size( PNN, 1 ) ~= size( PNN, 2 ) || isempty( PNN )
   error( 'Ambiguity covariance matrix (PNN) is not square or is empty' );
end


%% Compute and Return the ADOP

adop = sqrt( det( PNN ) )^(1/size(PNN,1));
