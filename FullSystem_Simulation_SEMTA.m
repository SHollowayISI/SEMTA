%% SEMTA Radar System
%{

    Sean Holloway
    SEMTA (Sea-Skimming Missile Tracking Radar Array) System
    MATLAB Simulation & Processing

    This shell file runs successive scripts and gauges progress.

    TODO: Collect save function in single method
    TODO: Main Loop: Move method options to simsetup

    TODO: Detection: Adjust location calculation?
    TODO: Detection: Implement bin combination?
    TODO: RadarScenario: Implement SNR calculation
    TODO: Tracking: Implement Kalman filtering
    TODO: Tracking: Implement Kalman filter
    
%}


%% Housekeeping
clear variables
close all
addpath(genpath(pwd));
tic

%% Definitions

nm = 1852;                      % Nautical miles in meters
c = physconst('LightSpeed');    % Speed of light in m/s

%% User Options

% Format to save figure files
save_format = {'.png', '.fig'};

% Save filename
filename = 'SingleTest_SEMTA';

% Rate to divide up fast time x slow time processing rates
sim_rate = 2^9;
% Optimized in this project for 2^9

%% Loop Through Files

% Run radar scenario setup
ExampleScenario_SEMTA

%% Run Simulation

% Perform main loop of simulation
MainLoop_SEMTA


%% Save Figures and Data

%{
% Establish file name
save_name = [filename, '_', datestr(now, 'mmddyy_HHMM')];

mat_path = 'MAT Files\Scenario Objects\';
fig_path = ['Figures\', save_name, '\'];

% Save scenario object
SaveScenario(scenario, save_name, mat_path);

% Save open figures
for ftype = 1:length(save_format)
    SaveFigures(save_name, fig_path, save_format{ftype});
end
%}





