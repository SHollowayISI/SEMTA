%% SEMTA Radar System - Target Initialization File
%{

    Sean Holloway
    SEMTA Target Init File
    
    This file specifies the properties of the target used in the SEMTA 
    simulation system.

    Use script 'FullSystem.m' to run scenarios.
    
%}

%% Target RCS Setup

% Options setup
scenario.rcs = struct( ...
    ...
    ... % RCS options
    'rcs_model',    'constant', ...        % Set 'model' or 'constant'
    'ave_rcs',      rcs_in, ...            % Target RCS in dBm^2
    ...
    ... % Model options
    'dim',      [6; 3; 0], ...          % [x; y; z] size of target
    'n_sc',     50, ...                 % Number of point scatterers
    'res_a',    0.5, ...                % Angle resolution in degrees
    'freq',     9e9:10e6:10e9, ...      % Frequency range to model
    ...
    ... % Broadside specular options
    'specular_on',  true, ...           % T/F Model broadside specular
    'theta_zero',   2, ...              % Angle at which spec. response is 1m^2
    'theta_ave',    30, ...             % Angle at which spec. meets statistical level
    'peak_rcs',     15);                % RCS at broadside specular peak

% Run RCS model
scenario.rcs = TargetRCSModel(scenario.rcs);

% View RCS results
% viewRCSFreq(scenario);
% viewRCSAng(scenario);

%% Target Trajectory Setup

% Options setup
scenario.traj = struct( ...
    ...
    ... % Trajectory model options
    'alt',      30, ...                % Altitude in meters
    'yvel',     100, ...                % Along track velocity in m/s
    'exc',      100, ...                % Excursion distance in meters
    'per',      0.1, ...                % Excursion period (Nominally 0.05 to 0.2)
    ...
    ... % Static options
    'pos_st',   range_in * [cosd(ang_in); sind(ang_in); 0], ...          % Position input if 'static' is used
    'vel_st',   [vel_in; 0; 0], ...          % Velocity input if 'static' is used
    ...
    ... % Model options
    'model',    'static');              % Set 'static', 'model', 'linear', or 'model_constant'

% Run Trajectory model
scenario.traj = TrajectoryModel(scenario.traj);

% View Trajectory
% viewTraj(scenario);
