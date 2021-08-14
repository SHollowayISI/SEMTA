%% SEMTA Radar System
%{

    Sean Holloway
    SEMTA (Sea-Skimming Missile Tracking Radar Array) System
    MATLAB Simulation & Processing

    This shell file runs successive scripts and gauges progress.

    TODO:
        - Error study
        - Test mode shell
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

%% Test Setup

filename = 'Results/Error Tests/ErrorTestRange2.mat';

if isfile(filename)
    
    % Load in variables
    load(filename);
    
    r_start = r_ind;
    a_start = a_ind;
    rcs_start = rcs_ind;
    
else

    iterations = 5;
    % iterations = 1;
    
    % range_list = 500:500:7000;
    range_list = [500, 6500];
    % ang_list = 0:15:45;
    ang_list = 0;
    vel_set = 0;
    % rcs_list = 0:-10:-20;
    rcs_list = 0;
    
    range_var = 10;
    ang_var = 0;
    vel_var = 0;
    
    range_err = nan(length(range_list), length(ang_list), length(rcs_list), iterations);
    ang_err = nan(length(range_list), length(ang_list), length(rcs_list), iterations);
    vel_err = nan(length(range_list), length(ang_list), length(rcs_list), iterations);
    snr_meas = nan(length(range_list), length(ang_list), length(rcs_list), iterations);
    snr_calc = nan(length(range_list), length(ang_list), length(rcs_list), iterations);
    
    range_meas = range_err;
    ang_meas = ang_err;
    vel_meas = vel_err;
    
    range_true = range_err;
    ang_true = ang_err;
    vel_true = vel_err;
    
    r_start = 0;
    a_start = 0;
    rcs_start = 0;

end

% Testing Loop
for r_ind = 1:length(range_list)
    if r_ind < r_start
        continue
    end
    for a_ind = 1:length(ang_list) 
        if a_ind < a_start
            continue
        end
        for rcs_ind = 1:length(rcs_list) 
            if rcs_ind < rcs_start
                continue
            end
            for iter = 1:iterations
                
                % Pass in test variables, plus randomness
                range_in = range_list(r_ind) + range_var * (2*rand(1)-1);
                ang_set = ang_list(a_ind);
                ang_in = ang_list(a_ind) + ang_var * (2*rand(1)-1);
                vel_in = vel_set + vel_var * (2*rand(1)-1);
                rcs_in = rcs_list(rcs_ind);

                %% Initialize Scenario Object
                
                % Initialization
                scenario = RadarScenario;
                
                %% Setup Structures for Simulation
                
                % Set up simulation parameters
                SetupSimulation_Test
                
                % Set up target properties
                SetupTarget_Test
                
                % Set up multistatic scenario
                SetupMulti_Test
                
                % Set up transciever and channel parameters
                SetupRadarScenario_Test
                
                %% Run Simulation
                
                % Perform main loop of simulation, signal and data processing
                Main_Test
                
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
            
            % Save to file every set of iterations
            save(filename, ...
                'range_list', 'range_true', 'range_meas', 'range_err', 'range_var', ...
                'ang_list', 'ang_true', 'ang_meas', 'ang_err', 'ang_var', ...
                'vel_set', 'vel_true', 'vel_meas', 'vel_err', 'vel_var', ...
                'rcs_list', 'snr_meas', 'snr_calc', ...
                'iterations', 'r_ind', 'a_ind', 'rcs_ind');
            
        end
    end
end


%% Save Figures and Data

% Run all end-of-simulation tasks
EndProcess




