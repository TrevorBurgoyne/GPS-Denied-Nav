%% Test Trajectory 1
% GPS Denied Navigation, AEM 4331 Fall 2022

function [pos, orient, vel, acc, angvel, times, wps, t] = trajectory1(Fs)
    % waypointTrajectory for a straight (no y motion), 20 min test trajectory.
    %   USAGE: `[position,orientation,velocity,acceleration,angularVelocity,wps,t] = trajectory1(Fs);` 
    %   will initialize a waypointTrajectory object with the pre-defined waypoints in this
    %   file, with Fs being the sample rate (Hz). wps is a list of the
    %   waypoints and t contains the corresponding times when those
    %   waypoints are reached. `pos, orient, vel, acc, angvel, times`
    %   will have the full trajectory details at each `times` entry.

    [wps1, t1] = launch_trajectory(); % start with the launch
    x0 = wps1(end,1);  % m, trajectory starting x position
    z0 = wps1(end,3);  % m, trajectory starting z position
    t0 = t1(end);      % s, trajectory start time (end of launch phase)
    tf = 20*60;        % s, 20 min flight
    v  = 100;          % m/s, steady flight speed
    xf = x0+v*(tf-t0); % m, final x position

    % Waypoints (m)
    wps2 = [xf, 0, z0];
    wps = [wps1', wps2']'; % append to launch
    
    % Times (s)
    t2  = [tf];
    t = [t1', t2']'; % append to launch
    
    traj = waypointTrajectory(wps, t, 'SampleRate', Fs);
    times = t(1):1/Fs:t(end);
    [pos, orient, vel, acc, angvel] = lookupPose(traj, times);
end

% waypointTrajectory Waypoint trajectory generator
%     TRAJ = waypointTrajectory(POINTS, T) returns a System object, TRAJ,
%     that generates a trajectory based on the specified waypoints POINTS and
%     times T. POINTS is an N-by-3 matrix that specifies the positions along
%     the trajectory. T is an N-element vector that specifies the times at
%     which the trajectory crosses the corresponding waypoints.
%  
%     TRAJ = waypointTrajectory(..., 'Name', value) returns a
%     waypointTrajectory System object by specifying its properties as
%     name-value pair arguments.  Unspecified properties have default values.
%     See the list of properties below.
%  
%     Step method syntax:
%  
%     [POS, ORIENT, VEL, ACC, ANGVEL] = step(TRAJ) outputs a frame of 
%     trajectory data based on the specified waypoints.
%  
%     The outputs of waypointTrajectory are defined as follows:
%  
%         POS       Position in the local navigation coordinate system 
%                   specified as a real finite N-by-3 array in meters. N is
%                   specified by the SamplesPerFrame property.
%  
%         ORIENT    Orientation with respect to the local navigation 
%                   coordinate system specified as a quaternion N-element
%                   column vector or a 3-by-3-by-N rotation matrix. Each
%                   quaternion or rotation matrix is a frame rotation from
%                   the local navigation coordinate system to the current
%                   body coordinate system. N is specified by the
%                   SamplesPerFrame property.
%  
%         VEL       Velocity in the local navigation coordinate system 
%                   specified as a real finite N-by-3 array in meters per
%                   second. N is specified by the SamplesPerFrame property.
%  
%         ACC       Acceleration in the local navigation coordinate system 
%                   specified as a real finite N-by-3 array in meters per
%                   second squared. N is specified by the SamplesPerFrame
%                   property.
%  
%         ANGVEL    Angular velocity in the local navigation coordinate 
%                   system specified as a real finite N-by-3 array in radians
%                   per second. N is specified by the SamplesPerFrame
%                   property.
%  
%     System objects may be called directly like a function instead of using
%     the step method. For example, y = step(obj, x) and y = obj(x) are
%     equivalent.
%  
%     waypointTrajectory methods:
%  
%     step               - See above description for use of this method
%     lookupPose         - Return pose information for a given set of times
%     perturbations      - Define perturbations to the trajectory
%     perturb            - Apply perturbations to the trajectory
%     release            - Allow property value and input characteristics to 
%                          change, and release waypointTrajectory resources
%     clone              - Create waypointTrajectory object with same property 
%                          values
%     isLocked           - Display locked status (logical)
%     reset              - Reset the states of the waypointTrajectory
%     isDone             - True if entire trajectory has been output
%  
%     waypointTrajectory construction properties:
%  
%     Waypoints          - Waypoints in the navigation frame (m)
%     TimeOfArrival      - Time at each waypoint (s)
%     Velocities         - Velocities at each waypoint (m/s)
%     Course             - Horizontal direction of travel (degrees)
%     GroundSpeed        - Groundspeed at each waypoint (m/s)
%     ClimbRate          - Climbrate at each waypoint (m/s)
%     Orientation        - Orientation at each waypoint
%     AutoPitch          - Align orientation (pitch) with direction of travel
%     AutoBank           - Align orientation (roll) to counteract centripetal force
%     ReferenceFrame     - Axes convention ('NED' or 'ENU')
%  
%     waypointTrajectory tunable properties:
%  
%     SampleRate      - Sample rate of trajectory (Hz)
%     SamplesPerFrame - Number of samples in the output
%  
%     % EXAMPLE 1: Visualize specified waypoints and computed position.
%     % Record the generated position and verify that it passes through the
%     % specified waypoints.
%     Fs = 50;
%     wps = [0 0 0;
%            1 0 0;
%            1 1 0;
%            1 2 0;
%            1 3 0];
%     t = 0:(size(wps,1)-1);
%     
%     % create trajectory
%     traj = waypointTrajectory(wps, t, 'SampleRate', Fs);
%     
%     % lookup pose information for entire trajectory
%     pos = lookupPose(traj, t(1):1/Fs:t(end));
%  
%     % Plot generated positions and specified waypoints.
%     plot(pos(:,1),pos(:,2), wps(:,1),wps(:,2), '--o')
%     title('Position')
%     xlabel('X (m)')
%     ylabel('Y (m)')
%     zlabel('Z (m)')
%     legend({'Position', 'Waypoints'})
%  
%     % EXAMPLE 2: Generate a racetrack trajectory by specifying the velocity
%     % and orientation at each waypoint.
%     Fs = 100;
%     wps = [0 0 0;
%            20 0 0;
%            20 5 0;
%            0 5 0;
%            0 0 0];
%     t = cumsum([0 10 1.25*pi 10 1.25*pi]).';
%     vels = [2 0 0;
%            2 0 0;
%            -2 0 0;
%            -2 0 0;
%            2 0 0];
%     eulerAngs = [0 0 0;
%                  0 0 0;
%                  180 0 0;
%                  180 0 0;
%                  0 0 0];
%     q = quaternion(eulerAngs, 'eulerd', 'ZYX', 'frame');
%     
%     traj = waypointTrajectory(wps, 'SampleRate', Fs, ...
%         'TimeOfArrival', t, 'Velocities', vels, 'Orientation', q);
%     
%     % fetch pose information one buffer frame at a time
%     [pos, orient, vel, acc, angvel] = traj();
%     i = 1;
%     spf = traj.SamplesPerFrame;
%     while ~isDone(traj)
%         idx = (i+1):(i+spf);
%         [pos(idx,:), orient(idx,:), ...
%             vel(idx,:), acc(idx,:), angvel(idx,:)] = traj();
%         i = i+spf;
%     end
%     % Plot generated positions and specified waypoints.
%     plot(pos(:,1),pos(:,2), wps(:,1),wps(:,2), '--o')
%     title('Position')
%     xlabel('X (m)')
%     ylabel('Y (m)')
%     zlabel('Z (m)')
%     legend({'Position', 'Waypoints'})
%     axis equal