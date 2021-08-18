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

filename = 'Results/Error Tests/ErrorTestAoASecondRun.mat';

if isfile(filename)
    
    % Load in variables
    load(filename);
    
    r_start = r_ind;
    a_start = a_ind;
    rcs_start = rcs_ind;
    
else
    
    iterations = 50;
    
    range_set = 3000;
    ang_list = 0:15:60;
    offset_list = 0:0.5:3;
%     ang_list = 45;
%     offset_list = 3;
    vel_set = 0;
    rcs_set = 0;
    
    range_var = 6;
    ang_var = 0;
    vel_var = 0;
    
    range_err = nan( length(ang_list), length(offset_list), iterations);
    ang_err = nan( length(ang_list), length(offset_list), iterations);
    vel_err = nan( length(ang_list), length(offset_list), iterations);
    snr_meas = nan( length(ang_list), length(offset_list), iterations);
    snr_calc = nan( length(ang_list), length(offset_list), iterations);
    
    range_meas = range_err;
    ang_meas = ang_err;
    vel_meas = vel_err;
    
    range_true = range_err;
    ang_true = ang_err;
    vel_true = vel_err;
    
    ang_start = 0;
    off_start = 0;
    
end

% Testing Loop
for ang_ind = 1:length(ang_list)
    if ang_ind < ang_start
        continue
    end
    for off_ind = 1:length(offset_list)
        if off_ind < off_start
            continue
        end
        
        for iter = 1:iterations
            
            % Pass in test variables, plus randomness
            range_in = range_set + range_var * (2*rand(1)-1);
            ang_set = ang_list(ang_ind);
            ang_in = ang_list(ang_ind) + offset_list(off_ind);
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
                
                range_true(ang_ind, off_ind, iter) = range_in;
                ang_true(ang_ind, off_ind, iter) = ang_in;
                vel_true(ang_ind, off_ind, iter) = vel_in;
                
                range_meas(ang_ind, off_ind, iter) = det_list.range(max_ind);
                ang_meas(ang_ind, off_ind, iter) = det_list.az(max_ind);
                vel_meas(ang_ind, off_ind, iter) = det_list.vel(max_ind);
                
                range_err(ang_ind, off_ind, iter) = det_list.range(max_ind) - range_in;
                ang_err(ang_ind, off_ind, iter) = det_list.az(max_ind) - ang_in;
                vel_err(ang_ind, off_ind, iter) = det_list.vel(max_ind) - vel_in;
                snr_meas(ang_ind, off_ind, iter) = det_list.SNR(max_ind);
                snr_calc(ang_ind, off_ind, iter) = CalculateSNR(scenario);
            end
            
        end
        
        % Save to file every set of iterations
        save(filename, ...
            'range_set', 'range_true', 'range_meas', 'range_err', 'range_var', ...
            'ang_list', 'ang_true', 'ang_meas', 'ang_err', 'ang_var', ...
            'vel_set', 'vel_true', 'vel_meas', 'vel_err', 'vel_var', ...
            'offset_list', 'rcs_set', 'snr_meas', 'snr_calc', ...
            'iterations', 'ang_ind', 'off_ind');
        
    end
end


%% Save Figures and Data

% Run all end-of-simulation tasks
EndProcess




