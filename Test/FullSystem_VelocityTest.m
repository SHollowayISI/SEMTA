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
        - Correct power law for angle estimation centroid

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

filename = 'Results/Error Tests/FixedWindowVelTest.mat';

if isfile(filename)
    
    % Load in variables
    load(filename);
    
    range_start = range_ind;
    ang_start = ang_ind;
    rcs_start = rcs_ind;
    vel_start = vel_ind;
    
else
    
    iterations = 20;
    
    range_list = 1000:1000:7000;
    %     res = 5.99584916;
    %     start = res*round(4000 / res);
    %     range_list = start + res*(1/64)*((1:64)-1);
    %     range_list = 200:200:7200;
    ang_list = 0:15:60;
    %     ang_list = 0;
    vel_list = -150:50:150;
    rcs_list = 0:-10:-20;
    %     rcs_list = 20;
    
    range_var = 6;
    ang_var = 0;
    vel_var = 1;
    
    range_err = nan( length(vel_list), length(range_list), length(ang_list), length(rcs_list), iterations);
    ang_err = nan( length(vel_list), length(range_list), length(ang_list), length(rcs_list), iterations);
    vel_err = nan( length(vel_list), length(range_list), length(ang_list), length(rcs_list), iterations);
    snr_meas = nan( length(vel_list), length(range_list), length(ang_list), length(rcs_list), iterations);
    snr_calc = nan( length(vel_list), length(range_list), length(ang_list), length(rcs_list), iterations);
    
    range_meas = range_err;
    ang_meas = ang_err;
    vel_meas = vel_err;
    
    range_true = range_err;
    ang_true = ang_err;
    vel_true = vel_err;
    
    rcs_start = 0;
    ang_start = 0;
    range_start = 0;
    vel_start = 0;
    
end


% Testing Loop
for rcs_ind = 1:length(rcs_list)
    
    if rcs_ind < rcs_start
        continue
    else
        rcs_start = 1;
    end
    
    for ang_ind = 1:length(ang_list)
        
        if ang_ind < ang_start
            continue
        else
            ang_start = 1;
        end
        
        for range_ind = 1:length(range_list)
            
            if range_ind < range_start
                continue
            else
                range_start = 1;
            end
            
            for vel_ind = 1:length(vel_list)
                
                if vel_ind < vel_start
                    continue
                else
                    vel_start = 1;
                end
                
                % Break out if previous iteration failed detection
                if (rcs_ind > 1) && (sum(isnan(snr_meas(vel_ind, range_ind, ang_ind, rcs_ind-1, :))) == iterations)
                    continue
                elseif (ang_ind > 1) && (sum(isnan(snr_meas(vel_ind, range_ind, ang_ind-1, rcs_ind, :))) == iterations)
                    continue
                elseif (range_ind > 1) && (sum(isnan(snr_meas(vel_ind, range_ind-1, ang_ind, rcs_ind, :))) == iterations)
                    continue
                elseif (vel_ind > 1) && (sum(isnan(snr_meas(vel_ind-1, range_ind, ang_ind, rcs_ind, :))) == iterations)
                    continue
                end
                
                
                for iter = 1:iterations
                    
                    % Pass in test variables, plus randomness
                    range_in = range_list(range_ind) + range_var * (2*rand(1)-1);
                    ang_set = ang_list(ang_ind);
                    ang_in = ang_list(ang_ind);
                    vel_in = vel_list(vel_ind) + vel_var * (2*rand(1)-1);
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
                        
                        range_true(vel_ind, range_ind, ang_ind, rcs_ind, iter) = range_in;
                        ang_true(vel_ind, range_ind, ang_ind, rcs_ind, iter) = ang_in;
                        vel_true(vel_ind, range_ind, ang_ind, rcs_ind, iter) = vel_in;
                        
                        range_meas(vel_ind, range_ind, ang_ind, rcs_ind, iter) = det_list.range(max_ind);
                        ang_meas(vel_ind, range_ind, ang_ind, rcs_ind, iter) = det_list.az(max_ind);
                        vel_meas(vel_ind, range_ind, ang_ind, rcs_ind, iter) = det_list.vel(max_ind);
                        
                        range_err(vel_ind, range_ind, ang_ind, rcs_ind, iter) = det_list.range(max_ind) - range_in;
                        ang_err(vel_ind, range_ind, ang_ind, rcs_ind, iter) = det_list.az(max_ind) - ang_in;
                        vel_err(vel_ind, range_ind, ang_ind, rcs_ind, iter) = det_list.vel(max_ind) - vel_in;
                        snr_meas(vel_ind, range_ind, ang_ind, rcs_ind, iter) = det_list.SNR(max_ind);
                        snr_calc(vel_ind, range_ind, ang_ind, rcs_ind, iter) = CalculateSNR(scenario);
                    end
                    
                end
                
                % Save to file every set of iterations
                save(filename, ...
                    'range_list', 'range_true', 'range_meas', 'range_err', 'range_var', ...
                    'ang_list', 'ang_true', 'ang_meas', 'ang_err', 'ang_var', ...
                    'vel_list', 'vel_true', 'vel_meas', 'vel_err', 'vel_var', ...
                    'rcs_list', 'snr_meas', 'snr_calc', ...
                    'iterations', 'range_ind', 'ang_ind', 'vel_ind', 'rcs_ind');
                
            end
        end
    end
end


%% Save Figures and Data

% Run all end-of-simulation tasks
EndProcess




