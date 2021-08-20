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

%% Test Setup

filename = 'Results/Error Tests/MonopulseTestDelete1.mat';

if isfile(filename)
    
    % Load in variables
    load(filename);
    
    offset_start = offset_ind;
    range_start = range_ind;
    ang_start = ang_ind;
    
else
    
    iterations = 20;
    
    range_list = 4000;
    ang_list = 45;
    offset_list = 0:0.5:3.5;
    vel_set = 0;
    rcs_set = 0;
    
    range_var = 6;
    ang_var = 0;
    vel_var = 0;
    
    range_err = nan(length(offset_list), length(range_list), length(ang_list), iterations);
    ang_err = nan(length(offset_list), length(range_list), length(ang_list), iterations);
    vel_err = nan(length(offset_list), length(range_list), length(ang_list), iterations);
    snr_meas = nan(length(offset_list), length(range_list), length(ang_list), iterations);
    snr_calc = nan(length(offset_list), length(range_list), length(ang_list), iterations);
    
    range_meas = range_err;
    ang_meas = ang_err;
    vel_meas = vel_err;
    
    range_true = range_err;
    ang_true = ang_err;
    vel_true = vel_err;
    
    offset_start = 0;
    ang_start = 0;
    range_start = 0;
    
end

% Testing Loop
for ang_ind = 1:length(ang_list)
    
    if ang_ind < ang_start
        continue
    end
    
    for range_ind = 1:length(range_list)
        
        if range_ind < range_start
            continue
        end
        
        for offset_ind = 1:length(offset_list)
            
            if offset_ind < offset_start
                continue
            end
            
            % Break out if previous iteration failed detection
            if (offset_ind > 1) && (sum(isnan(snr_meas(offset_ind-1, range_ind, ang_ind, :))) > iterations/2)
                continue
            elseif (ang_ind > 1) && (sum(isnan(snr_meas(offset_ind, range_ind, ang_ind-1, :))) > iterations/2)
                continue
            elseif (range_ind > 1) && (sum(isnan(snr_meas(offset_ind, range_ind-1, ang_ind, :))) > iterations/2)
                continue
            end
            
            
            for iter = 1:iterations
                
                % Pass in test variables, plus randomness
                range_in = range_list(range_ind) + range_var * (2*rand(1)-1);
                ang_set = ang_list(ang_ind);
                ang_in = ang_list(ang_ind) + offset_list(offset_ind);
                vel_in = vel_set + vel_var * (2*rand(1)-1);
                rcs_in = rcs_set;
                
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
                    
                    range_true(offset_ind, range_ind, ang_ind, iter) = range_in;
                    ang_true(offset_ind, range_ind, ang_ind, iter) = ang_in;
                    vel_true(offset_ind, range_ind, ang_ind, iter) = vel_in;
                    
                    range_meas(offset_ind, range_ind, ang_ind, iter) = det_list.range(max_ind);
                    ang_meas(offset_ind, range_ind, ang_ind, iter) = det_list.az(max_ind);
                    vel_meas(offset_ind, range_ind, ang_ind, iter) = det_list.vel(max_ind);
                    
                    range_err(offset_ind, range_ind, ang_ind, iter) = det_list.range(max_ind) - range_in;
                    ang_err(offset_ind, range_ind, ang_ind, iter) = det_list.az(max_ind) - ang_in;
                    vel_err(offset_ind, range_ind, ang_ind, iter) = det_list.vel(max_ind) - vel_in;
                    snr_meas(offset_ind, range_ind, ang_ind, iter) = det_list.SNR(max_ind);
                    snr_calc(offset_ind, range_ind, ang_ind, iter) = CalculateSNR(scenario);
                end
                
            end
            
            % Save to file every set of iterations
            save(filename, ...
                'offset_list', ...
                'range_list', 'range_true', 'range_meas', 'range_err', 'range_var', ...
                'ang_list', 'ang_true', 'ang_meas', 'ang_err', 'ang_var', ...
                'vel_set', 'vel_true', 'vel_meas', 'vel_err', 'vel_var', ...
                'rcs_set', 'snr_meas', 'snr_calc', ...
                'iterations', 'range_ind', 'ang_ind', 'offset_ind');
            
        end
    end
end


%% Save Figures and Data

% Run all end-of-simulation tasks
EndProcess




