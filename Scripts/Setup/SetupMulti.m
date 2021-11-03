%% SEMTA Radar System - Target Initialization File
%{

    Sean Holloway
    SEMTA Multistatic Init File
    
    This file specifies the parameters of multistatic simulation for the
    SEMTA project.

    Use script 'FullSystem.m' to run scenarios.
    
%}

%% Definitions

% Nautical mile in meters
nm = 1852;

%% Multistatic Scenario Setup

% Locations of radar units
num_receivers = 3;                      % Number of radar units
dist_from_center    = 5000;             % Cross-track distance from track to radar units
unit_spacing        = 1024;             % Spacing between units
spacing_offset      = -1024;            % Along-track offset of units

radar_pos = ...
    [-dist_from_center  * ones(1,num_receivers); ...     % Constant x location
     spacing_offset + ...
     (unit_spacing      * ((0:(num_receivers-1)))); ...  % Incremental y distance
                          zeros(1,num_receivers)];       % Constant z elevation
                      
% Timing setup
simulation_time = 5;                   % Amount of time to simulate, in seconds
timing_jitter = 0.02;                    % Maximum random deviation between unit frame times

% Multistatic properties
scenario.multi = struct( ...
    ...
    'sim_time',     simulation_time, ...
    'start_time',   timing_jitter * rand(num_receivers, 1), ...
    'n_re',         num_receivers, ...
    'radar_pos',    radar_pos);

%% Run Setup Scripts

% Allocate data structures
multiSetup(scenario);




