%% SEMTA Radar System - Example Simulation Initialization File
%{

    Sean Holloway
    SEMTA Simulation Init File
    
    This file specifies simulation parameters for SEMTA simulation.

    Use script 'FullSystem.m' to run scenarios.
    
%}

%% Simulation Parameter Setup

save_format.list = {'.png','.fig'};

% Radar simulation and processing setup
scenario.simsetup = struct( ...
    ...
    ... % Simulation Properties
    'readout',      true, ...                   % Read out target data T/F
    ...
    'clear_cube',   true, ...
    'send_alert',   false, ...                  % Send email alert T/F
    'attach_zip',   false, ...
    'alert_address', 'hollowayseanm@gmail.com', ...
    ...                                         % Email address for status updates
    'filename',     'Error_SEMTA', ...          % Filename to save data as
    'save_format',  save_format, ...            % File types to save figures
    'save_figs',    false, ...                  % Save figures T/F
    'save_mat',     false, ...                  % Save mat file T/F
    'reduce_mat',   false, ...                  % Reduce mat file for saving
    'save_track',   false);                      % Save tracking T/F


