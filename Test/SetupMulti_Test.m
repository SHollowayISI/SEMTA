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
% num_receivers = 3;
% radar_pos = ...
%     [-0.5 * nm * ones(1,num_receivers); ...    % Constant x location
%      0.25 * nm * ((0:(num_receivers-1))); ...  % Incremental y distance
%         0 * nm * ones(1,num_receivers)];       % Constant z elevation

% radar_pos = [-4000; 0; 0];
% dist_from_center = 0.5 * nm;
% unit_spacing = 0.25 * nm;

dist_from_center = dist_in;
unit_spacing = spacing_in;

start = spacing_in * rand(1);
v = scenario.traj.yvel;

stop = start + frames_in * v * 0.0512;
early_unit = 1 - round(start / unit_spacing);
spacing_offset = - start - early_unit*unit_spacing;
num_receivers = 2 + round(stop / unit_spacing) + early_unit;

radar_pos = ...
    [-dist_from_center  * ones(1,num_receivers); ...     % Constant x location
     spacing_offset + ...
     (unit_spacing      * ((0:(num_receivers-1)))); ...  % Incremental y distance
                          zeros(1,num_receivers)];       % Constant z elevation

% Multistatic properties
scenario.multi = struct( ...
    ...
    'n_fr',         frames_in, ...                  % Number of frames for simulation
    'n_re',         size(radar_pos,2), ...  % Number of radar units to simulate
    'radar_pos',    radar_pos);

%% Run Setup Scripts

% Allocate data structures
multiSetup(scenario);




