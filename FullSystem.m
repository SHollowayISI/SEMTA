%% SEMTA Radar System
%{

    Sean Holloway
    SEMTA (Sea-Skimming Missile Tracking Radar Array) System
    MATLAB Simulation & Processing

    This shell file runs successive scripts and gauges progress.

    TODO:
        - Error study
        - Multistatic processing
        - Save detection coordinates


    Notes to self:
        - Frequency hopping can be implemented via FrequencyOffset port on
        linearFMwaveform object
        - Similarly, PRF changing can be implemented via an input port on
        waveform object
    
%}

%% Housekeeping, Timing, and Path Management

% Add current folders to path
addpath(genpath(pwd));

% Run all start-of-process tasks
StartProcess;

%% Initialize Scenario Object

iterations = 20;
% iterations = 1;

range_list = 500:500:7500;
% ang_list = 0:15:45;
ang_list = 0;
vel_set = 0;
% rcs_list = 0:-10:-20;
rcs_list = 0;

range_var = 6;
ang_var = 1;
vel_var = 1;

range_err = nan(length(range_list), length(ang_list), length(rcs_list), iterations);
ang_err = nan(length(range_list), length(ang_list), length(rcs_list), iterations);
vel_err = nan(length(range_list), length(ang_list), length(rcs_list), iterations);
snr_meas = nan(length(range_list), length(ang_list), length(rcs_list), iterations);
snr_calc = nan(length(range_list), length(ang_list), length(rcs_list), iterations);

for r_ind = 1:length(range_list)
    for a_ind = 1:length(ang_list)
        for rcs_ind = 1:length(rcs_list)
            for iter = 1:iterations
                
                range_in = range_list(r_ind) + range_var * (2*rand(1)-1);
                ang_set = ang_list(a_ind);
                ang_in = ang_list(a_ind) + ang_var * (2*rand(1)-1);
                vel_in = vel_set + vel_var * (2*rand(1)-1);
                rcs_in = rcs_list(rcs_ind);

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
                Main
                
                det_list = scenario.detection.detect_list;
                if det_list.num_detect > 0
                    [~, max_ind] = max(det_list.SNR);
                    
                    range_true(r_ind, a_ind, rcs_ind, iter) = range_in;
                    ang_true(r_ind, a_ind, rcs_ind, iter) = ang_in;
                    vel_true(r_ind, a_ind, rcs_ind, iter) = vel_in;
                    
                    range_meas(r_ind, a_ind, rcs_ind, iter) = det_list.range(max_ind);
                    ang_meas(r_ind, a_ind, rcs_ind, iter) = det_list.az(max_ind);
                    vel_meas(r_ind, a_ind, rcs_ind, iter) = det_list.vel(max_ind);
                    
                    range_err(r_ind, a_ind, rcs_ind, iter) = det_list.range(max_ind) - range_in;
                    ang_err(r_ind, a_ind, rcs_ind, iter) = det_list.az(max_ind) - ang_in;
                    vel_err(r_ind, a_ind, rcs_ind, iter) = det_list.vel(max_ind) - vel_in;
                    snr_meas(r_ind, a_ind, rcs_ind, iter) = det_list.SNR(max_ind);
                    snr_calc(r_ind, a_ind, rcs_ind, iter) = CalculateSNR(scenario);
                end

            end
        end
    end
end

filename = 'Results/Error Tests/errorTestRangeSingleCPI';
save([filename, '.mat'], ...
    'range_list', 'range_true', 'range_meas', 'range_err', ...
    'ang_list', 'ang_true', 'ang_meas', 'ang_err', ...
    'vel_set', 'vel_true', 'vel_meas', 'vel_err', ...
    'rcs_list', 'snr_meas', 'snr_calc');

%% Save Figures and Data

% Run all end-of-simulation tasks
EndProcess




