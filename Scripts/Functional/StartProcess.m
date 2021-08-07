%% SEMTA - Housekeeping
%{

    Sean Holloway
    SEMTA Start of Process Actions
    
    Set up MATLAB environment for SEMTA system.
    
%}

%% Variable management

% Clear current variables
clear variables;

%% Figure management

% Close all current figures
close all;

%% Path management

% Add folders from home
addpath(genpath(pwd));

% Folders to exclude
exclude_paths = {'./Old', './References', './.git', './Scratch'};

% Remove excluded folders from path
for path = exclude_paths
    rmpath(genpath(path{:}));
end

%% Timing

% Start timer
tic;