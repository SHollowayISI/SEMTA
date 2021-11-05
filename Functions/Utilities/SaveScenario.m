function [] = SaveScenario(scenario, save_name, mat_path)
%SAVESCENARIO Saves Radar Scenario object
%   Writes Radar Scenario object to .mat file in mat_path folder with
%   save_name as filename

%% Save Scenario to .mat file

% Create directory if non-existant
if ~exist(mat_path, 'dir')
    mkdir(mat_path)
end

% Remove high-volume structures from scenario object
scenario.rx_sig = [];
scenario.cube = [];
scenario.detection = [];

% Set file path
filename = [mat_path, 'scenario_', save_name, '.mat'];

% Save .mat file
save(filename, 'scenario', '-v7.3')

% Display update to command window
disp(['Scenario object saved as ', filename]);

end

