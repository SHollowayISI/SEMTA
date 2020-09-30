%% SEMTA Radar System
%{

    Sean Holloway
    SEMTA (Sea-Skimming Missile Tracking Radar Array) System
    MATLAB Simulation & Processing

    This shell file runs successive scripts and gauges progress.

    TODO: Main Loop: Move method options to simsetup
    TODO: Tracking: Revise
    
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

% Email address for alerts
alert_address = 'sholloway@intellisenseinc.com';

% Choose whether to attach figures to email
attach_files = true;

% Rate to divide up fast time x slow time processing rates
sim_rate = 2^9;
% Optimized at 2^9 for this project

%% Loop Through Files

% Pull list of files from directory
dir_list = dir('Automated Testing/To Run/*.m');

for file_index = 1:length(dir_list)
    
    % Display current test
    disp(['Beginning Scenario: ', dir_list(file_index).name]);
    
    % Clear scenario object
    clear scenario
    
    % Pull filename from directory
    scenario_filename = dir_list(file_index).name;
    
    %% Setup Radar Scenario
    
    % Run radar scenario setup
    run(['Automated Testing/To Run/', scenario_filename])
    
    %% Run Simulation
    
    % Perform main loop of simulation
    MainLoop_SEMTA
    
    
    %% Save Figures and Data
    
    % Establish file name
    save_name = [scenario_filename(1:end-2), '_', datestr(now, 'mmddyy_HHMM')];
    
    mat_path = 'MAT Files\Scenario Objects\';
    fig_path = ['Figures\', save_name, '\'];
    
    % Save scenario object
    SaveScenario(scenario, save_name, mat_path);
    
    % Save open figures
    for ftype = 1:length(save_format)
        SaveFigures(save_name, fig_path, save_format{ftype});
    end
    
    % Close all figures after saving
    close all
    
    % Read elapsed time
    toc
    
        %% Send Email Alert
    
    % Set up email system
    EmailSetup();
    
    % Send email to alert address
    EmailAlert(alert_address, save_name, attach_files);
    
    %% File Management
    
    % Move file to "Complete" folder
    movefile(['Automated Testing/To Run/', scenario_filename], ...
        'Automated Testing/Complete/');
    
    % Display message to command window
    disp(['Simulation scenario complete: ', scenario_filename]);
    
end





