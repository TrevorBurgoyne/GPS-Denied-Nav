%========================================================%
%                case_study_2_wrapper.m
%
%   This m-script calls the various function required for
%   the second case study in Chapter 7.  
%
%  Programmer:      Demoz Gebre-Egziabher
%  Created on:      March 15, 2007
%  Last Modified:   Feb 11, 2009 
%
%========================================================%

clc;
close all;
clear all;

%   Define MATLAB Paths

if(ispc)
    addpath ..\..\gnss_ins_functions\;
else
    addpath ../../gnss_ins_functions/;
end    

%   Save Sensor Data Flag

SENSOR_SAVE_FLAG = 1;           %   Sensor data gets saved if = 1.
                                %   If = 0, sensor data is not saved.

%   Select the Appropriate Motion Profile

%TRAJECTORY = 'A' --- >     heading North at a constant speed and sinusoidally varying heading 
%TRAJECTORY = 'B' --- >     heading North at constant speed and heading
%TRAJECTORY = 'C' --- >     heading North at a sinusoidally varying heading and speed    
%TRAJECTORY = 'D' --- >     heading North at constant heading but sinusoidally varying speed

TRAJECTORY = 'A';

%   Generate the sensor and trajectory data

gen_traj_sensor_data(TRAJECTORY,SENSOR_SAVE_FLAG);


%   Select the Appropriate EKF FILTER_ARCHITECTURE
%   Note:  'H' = Heading derived from a Magnetometer/Compass.

%   'PVH'   --- >     Position, Velocity and Magnetometer/Compass Aiding
%   'PV'    --- >     Position and Velocity Aiding
%   'PH'    --- >     Position and Magnetometer/Compass Aiding
%   'P'     --- >     Position Aiding only

FILTER_ARCHITECTURE = 'P';

%   Define EKF Run-time and Saving Parameters

LINEARIZE_ABT_TRUE = 0;     %   If = 1, EKF is linearized about
                            %   the true state trajectory (Idealized
                            %   scenario).  Otherwise, linearization occurs
                            %   about the estimated trajectory (realistic
                            %   real-time scenario).
                            
FILTER_FLAG = 1;            %   If = 0, inertial navigator is run in an open
                            %   open loop fashion (i.e., no aiding).  When
                            %   = 1, the system integrates inertial sensor
                            %   derived navigation information with the
                            %   aiding source information.

%   Simulation Data Save Flag

FILTER_SAVE_FLAG = 1;           %   Navigation algorithm output data gets saved 
                                %   if = 1.  If = 0, navigation algorithm data 
                                %   is not saved.

%   Simulate EKF

gnss_ins_EKF_2D(TRAJECTORY,FILTER_ARCHITECTURE,LINEARIZE_ABT_TRUE,...
                                    FILTER_FLAG, FILTER_SAVE_FLAG);
                                    
 %   Plot Simulation Results
 
plot_EKF_results(TRAJECTORY,FILTER_ARCHITECTURE);
 