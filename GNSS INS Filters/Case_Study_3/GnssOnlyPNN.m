function P_NN = GnssOnlyPNN( satAzimuths, satElevations, Rcode, Rphase, wavelength )
%
% P_NN = GnssOnlyPNN( satAzimuths, satElevations, Rcode, Rphase, wavelength )
%
% SUMMARY: 
%    GnssOnlyPNN computes the covariance matrix of the carrier phase ambiguities using only
%    pseudorange and carrier phase GNSS data.  The results is based on least-squares positioning, 
%    and is therefore indicative of performance following a complete GNSS data outage.
%
% INPUT:
%  - satAzimuths: The N-vector of azimuth angles of the GNSS satellites being tracked in units of 
%                 degrees.
%  - satElevations: The N-vector of elevation angles of the GNSS satellites being tracked in units 
%                   of degrees.
%  - Rcode: The NxN covariance matrix of the undifferenced pseudorange measurements, or, if a scalar 
%           is provided, the covariance matrix is computed as the identity matrix scaled by the 
%           given value (i.e., the given value is assumed to be a variance).  All inputs should be 
%           in units of metres^2. 
%  - Rphase: The NxN covariance matrix of the undifferenced carrier phase measurements, or, if a 
%            scalar is provided, the covariance matrix is computed as the identity matrix scaled by 
%            the given value (i.e., the given value is assumed to be a variance).  All input should 
%            be in units of cycles^2. 
%  - wavelength: The wavelength of the carrier phase data in units of metres per cycle.
%
% OUTPUT:
%  - P_NN: The covariance matrix of the estimated double-difference carrier phase ambiguities.
%
% DESCRIPTION:
%    The function uses the followings steps:
%     - Use satellite azimuth and elevation angles to compute the direction cosine matrix to each
%       satellite being tracked.
%     - Use the direction cosine matrix from above to compute the design matrix (H) for the 
%       pseudorange and carrier phase observations, assuming the only states being estimated are the
%       three position states and the N-1 carrier phase ambiguities.
%     - Use the given covariance matrices, as well as the design matrix from the above step and 
%       compute the least-squares covariance matrix of the entire state vector.  From this matrix 
%       the sub-matrix corresponding to the ambiguity states is then extracted an returned.
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

% check that all elements have the proper size
if length( satAzimuths ) ~= length( satElevations )
   error( 'satAzimuths and satElevations must have the same length' );
end

if size( Rcode, 1 ) ~= size( Rcode, 2 ) 
   error( 'Pseudorange covariance matrix (Rcode) is not square or scalar' );
end

if size( Rphase, 1 ) ~= size( Rphase, 2 )
   error( 'Carrier phase covariance matrix (Rphase) is not square or scalar' );
end

if size( Rphase, 1 ) ~= size( Rphase, 2 )
   error( 'Carrier phase covariance matrix (Rphase) is not square or scalar' );
end

if ~isscalar( Rcode )
   if size( Rcode, 1 ) ~= length( satElevations )
      error( 'Code covariance matrix (Rcode) is not the same size as the number of satellites tracked' );
   end
end

if ~isscalar( Rphase )
   if size( Rphase, 1 ) ~= length( satElevations )
      error( 'Phase covariance matrix (Rphase) is not the same size as the number of satellites tracked' );
   end
end


%% Data Formatting

% get the number of satellites tracked
N = length( satAzimuths );

% form column vectors of azimuth and elevation and scale to untis of radians
satAzimuths = reshape( satAzimuths, N, 1 ) * pi / 180;
satElevations = reshape( satElevations, N, 1 ) * pi / 180;

% form covariance matrices, if necessary
if isscalar( Rcode )
   Rcode = eye(N) * Rcode;
end

if isscalar( Rphase )
   Rphase = eye(N) * Rphase;
end


%% Compute Direction Cosine Matrix

% direction cosine in North, East and Vertical frame
DCM = [ cos( satElevations ) .* cos( satAzimuths ) , cos( satElevations ) .* sin( satAzimuths ) , sin( satElevations ) ];


%% Compute the Double Difference Design Matrix and Covariance Matrix

% form the double differencing matrix
D = [ ones( N-1, 1 ) , -eye( N-1 ) ];

% design matrices
Hcode = [ D * DCM , zeros( N-1, N-1 ) ];
Hphase = [ D * DCM , wavelength * eye( N-1 ) ];
H = [ Hcode ; Hphase ];

Rcode = D * Rcode * D';
Rphase = D * Rphase * D';
R = blkdiag( Rcode, Rphase );


%% Compute the Least-Squares Covariance Matrix and Extract Ambiguity Sub-Matrix

P = inv( H' * inv(R) * H );

P_NN = P( 4:end, 4:end );






