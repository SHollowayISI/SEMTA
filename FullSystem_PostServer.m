%% SEMTA Radar System
%{

    Sean Holloway
    SEMTA (Sea-Skimming Missile Tracking Radar Array) System
    MATLAB Simulation & Processing

    This shell file runs successive scripts and gauges progress.
%}

%% Housekeeping, Timing, and Path Management

% Add current folders to path
addpath(genpath(pwd));

% Run all start-of-process tasks
StartProcess;

%% Initialize Scenario Object

% Initialization
scenario = RadarScenario;

%% Setup Structures for Simulation

% Set up simulation parameters
SetupSimulation

% Set up target properties
SetupTarget

% Set up multistatic scenario
SetupMulti

% Set up transciever and channel parameters
SetupRadarScenario

%% Run Simulation

% Perform main loop of simulation, signal and data processing
Main_PostServer

%% Save Figures and Data

% Run all end-of-simulation tasks
EndProcess




