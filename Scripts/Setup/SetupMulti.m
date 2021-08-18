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
num_receivers = 11;
% radar_pos = ...
%     [-0.5 * nm * ones(1,num_receivers); ...    % Constant x location
%      0.25 * nm * ((0:(num_receivers-1))); ...  % Incremental y distance
%         0 * nm * ones(1,num_receivers)];       % Constant z elevation

% radar_pos = [0; 0; 0];

dist_from_center = 1000;
unit_spacing = 500;
% dist_from_center = 0.5 * nm;
% unit_spacing = 0.25 * nm;

radar_pos = ...
    [-dist_from_center  * ones(1,num_receivers); ...     % Constant x location
     unit_spacing       * ((0:(num_receivers-1))); ...   % Incremental y distance
                          zeros(1,num_receivers)];       % Constant z elevation

% Multistatic properties
scenario.multi = struct( ...
    ...
    'n_fr',         1, ...                  % Number of frames for simulation
    'n_re',         size(radar_pos,2), ...  % Number of radar units to simulate
    'radar_pos',    radar_pos);
    
% Multistatic tracking properties
tracking = struct( ...
    ...
    'method',       'SNR', ...          % Method to select single unit results
    'dist_thresh',  Inf, ...            % Mahanalobis distance association threshold
    'miss_max',     2, ...              % Number of misses required to inactivate track
    'max_hits_fa',  1, ...              % Maximum number of hits for track to still be false alarm
    'EKF',          false, ...          % T/F use extended Kalman filter
    'sigma_v',      [4.5 4.5 4.5], ...  % XYZ target motion uncertainty
    'sigma_z',      [0.5 0.5 0.5 1]);   % XYZnull or RAEV measurement uncertainty

scenario.multi.tracking_multi = tracking;

%% Run Setup Scripts

% Allocate data structures
multiSetup(scenario);




