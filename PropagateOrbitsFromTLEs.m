function d = PropagateOrbitsFromTLEs( tleFile, timeVec, hasHeader )
% UNCOMMET LINE 97 TO TEST CODE
% OTHERWISE WILL TAKE *AWHILE* - IN
%
% Load a TLE file and propagate each satellite's orbit over the given time.
% Uses Vallado's Matlab software to perform SGP4 orbit propagation,
% available at:  https://celestrak.org/software/vallado-sw.php
%
% INPUTS:
%   tleFile   (:)     String name of TLE file to load, with "N" entries.
%   timeVec   (1,P)   Time vector in seconds to propagate all orbits.
%   hasHeader (1)     Boolean. If true, then we expect the TLE file to have
%                     a header line above each two-line element set. If
%                     false, then we expect no header line.
%
% OUTPUTS:
%   d     (.)         A data structure with fields:
%                     .jD0      Reference Julian date at time 0
%                     .dateVec  Reference date [Y M D h m s] at time 0
%                     .t        Time vector in seconds (same as timeVec)
%                     .rECI     {1,N} ECI position vector of each satellite
%                                     over time. Each is a 3 x P matrix
%                     .vECI     {1,N} ECI velocity vector of each satellite
%                                     over time. Each is a 3 x P matrix
%                     .rEF      {1,N} EF position vector of each satellite
%                                     over time. Each is a 3 x P matrix
%                     .vEF      {1,N} EF velocity vector of each satellite
%                                     over time. Each is a 3 x P matrix
%                     .arrayRef Random int ([1 mesh+1]) for reference point
%                                     on Earth to find satellites in view
%                     .rEF0     {1,3} On Earth Frame (rEF) reference point
%                     .E        {1,N} Angles of satellites from rEF
%                     .numSats  int of number of satellites in view of rEF
%
% A TLE file with headers would look like this:
%
% STARLINK-1007
% 1 44713U 19074A   22336.47699629  .00001505  00000+0  11993-3 0  9994
% 2 44713  53.0561 143.1715 0001798  75.7035 284.4154 15.06402874169073
% STARLINK-1008
% 1 44714U 19074B   22336.45583031 -.00000584  00000+0 -20324-4 0  9999
% 2 44714  53.0538 143.2539 0001534  80.4773 279.6389 15.06390559169075
% ...
%
% A TLE file withOUT headers would look like this:
%
% 1 44713U 19074A   22336.47699629  .00001505  00000+0  11993-3 0  9994
% 2 44713  53.0561 143.1715 0001798  75.7035 284.4154 15.06402874169073
% 1 44714U 19074B   22336.45583031 -.00000584  00000+0 -20324-4 0  9999
% 2 44714  53.0538 143.2539 0001534  80.4773 279.6389 15.06390559169075
% ...
%
% Authors: Joseph Mueller, 2022
%          Isaac Nistler, 2022
%          Justin Dang, 2022

if nargin<1
  tleFile = 'starlink.txt'; % assuming you have this file on your path
end

if nargin<2
  timeVec = 0 : 60 : 3*3600; % default, six hours at 30 second increments
end

if nargin<3
  hasHeader = true; % assume the TLE file has headers in it
end

s = fileread(tleFile);
lines = strsplit(s,'\n');

% for i=1:length(lines)
%   fprintf(1,'%s - %d\n',lines{i},length(lines{i}));
% end
% return

if hasHeader
  nTLE = ceil(length(lines)/3);
  index =2:3:(nTLE*3-1);
else
  nTLE = ceil(length(lines)/2);
  index =1:2:(nTLE*2-1);
end

d.t = timeVec;

%% Some constants needed for ECI / EF rotation later
conv = pi / (180*3600);
xp   = -0.140682 * conv;  % " to rad
yp   =  0.333309 * conv;
lod  =  0.0015563;  % sec
ddpsi = -0.052195 * conv;  % " to rad
ddeps = -0.003875 * conv;


%% for each TLE in the list...
%%nTLE = 10; % Hard Coded to Test
for k=1:nTLE

  if index(k)>length(lines)
    break;
  end

  % 1. Obtain the two-line element (TLE) of that satellite.
  line1 = lines{index(k)};
  line2 = lines{index(k)+1};

  %
  % 2. Propagate the orbit. Two ways of doing this:
  % 	a. Convert the TLE into orbital elements, and do a simple Keplerian
  %        propagation
  % 		- Once you have orbital elements, you can use RVFromCOE and
  %           then RVAtTFromR0V0, methods in the orbital zip file.
  % 		- I don’t have a method in the zip file to convert TLE into
  %           orbital elements, though
  % 	b. Propagate using the “SGP4” method, special-general perturbations.
  % 		- I have methods to do this, but they are all part of a
  %           propriety software package
  % 		- Alternatively, you can download free Matlab code from
  %           celestrak to do it.

  typerun = 'c'; % 
  opsmode= 'a';  % afspc approach
  whichconst = 72;
  eqeterms = 2;

  [~, ~, ~, satrec] = twoline2rv(line1, line2, typerun,'e', opsmode, whichconst);
  fprintf(1,' %d\n', satrec.satnum);

  % record the epoch as [JulianDate, FractionOfDay]
  epoch(k,:) = [satrec.jdsatepoch, satrec.jdsatepochf];

  % measure time with respect to epoch of first satellite in TLE list
  if k>1
    offsetMinutes = (epoch(1,1)-epoch(k,1))*1440 + (epoch(1,2)-epoch(k,2))*1440;
  else
    d.jDEpoch = satrec.jdsatepoch + satrec.jdsatepochf;
    offsetMinutes = 0.0;
  end
  adjustedTimeVec = timeVec/60 + offsetMinutes;

  % propagate using SGP4
  aecie = [0 0 0]'; % acceleration vector (not needed, set to zero)
  for j=1:length(timeVec)
    [~,d.rECI{k}(:,j),d.vECI{k}(:,j)] = sgp4 (satrec,  adjustedTimeVec(j));

    %
    % 3. Compute the satellite position in the EF frame.
    % 	- The orbit position data will initially be in an inertial frame.
    %     You need to convert to the "Earth-Fixed" or EF frame.
    % 	- This answers the question you had. Geocentric orbits are always
    %     described (at first) in an Earth-centered inertial (ECI) frame.
    %     The Earth rotates, of course, so any point on the Earth is always
    %     moving in the inertial frame. If you specify the date and time,
    %     there are methods that can tell you the rotation matrix between
    %     the ECI and the EF frame.
    % 	- The free Matlab code from celestrak has some methods to do this
    %     rotation.
    %
    jD = d.jDEpoch + adjustedTimeVec(j)/1440;
    ttt = (jD-2451545.0)/36525.0;
    [~,d.rEF{k}(:,j),d.vEF{k}(:,j)] = eci2ecef( d.rECI{k}(:,j), d.vECI{k}(:,j), ...
      aecie, ttt, jD, lod, xp, yp, eqeterms, ddpsi, ddeps );

  end

end

%% Plot all of the orbits in the ECI and EF frame
figure('name','Orbits in ECI')
for j=1:nTLE 
  plot3(d.rECI{j}(1,:),d.rECI{j}(2,:),d.rECI{j}(3,:)), hold on, 
end
axis equal, grid on
xlabel('X (km)'), ylabel('Y (km)'), zlabel('Z (km)')


figure('name','Orbits in EF')
for j=1:nTLE 
  plot3(d.rEF{j}(1,:),d.rEF{j}(2,:),d.rEF{j}(3,:)), hold on, 
end
axis equal, grid on
xlabel('X (Mm)'), ylabel('Y (Mm)'), zlabel('Z (Mm)')

%% 4. Reference Point for EF Frame
% Define your reference point in the EF frame.
% 	- This is where your system will be used. You may want to do an 
%     analysis that considers lots of different reference points. 
%     Pick one to start.

R = 6.271;
mesh = 500;
[X,Y,Z] = sphere(mesh);
X = X*R;
Y = Y*R;
Z = Z*R;

% Randomized Ref Point
d.arrayRef = randi([1 mesh+1]);
d.rEF0(1) = X(d.arrayRef,d.arrayRef);
d.rEF0(2) = Y(d.arrayRef,d.arrayRef);
d.rEF0(3) = Z(d.arrayRef,d.arrayRef);
plot3(d.rEF0(1),d.rEF0(2),d.rEF0(3),'.m','MarkerSize',20)
surf(X,Y,Z)

%% 5. Elevation Angle of Satellite across time
% Compute the elevation angle of the satellite across time
% 	- A satellite with an elevation above 0 deg is above the horizon, so it
%     is in view. Pick 5 deg as a more realistic threshold
%     (e.g. mountains, buildings)

for j=1:nTLE 
    for k=1:length(d.t)
        relPos = (d.rEF{j}(:,k) - d.rEF0') / norm(d.rEF{j}(:,k)-d.rEF0');
        up = d.rEF0' / norm(d.rEF0');
        X = acosd(dot(up,relPos));
        d.E(j) = 90-X;
    end
end

%% 6. Number of Satellites
% Now you have the data to count how many satellites are in
%     view at each time.
d.numSats = 0;
for j=1:nTLE
    if (d.E(j) >= 5) && (d.E(j) <= 175)
    d.numSats = d.numSats + 1;
    end
end

answer = [num2str(d.numSats), ' satellites are in view from the' ...
    ' Reference Point: (', num2str(d.rEF0(1)), ', ', num2str(d.rEF0(2)),...
    ', ', num2str(d.rEF0(3)), ')-Mm'];

disp(answer);