%% SEMTA Radar System
%{

    Sean Holloway
    SEMTA (Sea-Skimming Missile Tracking Radar Array) System
    MATLAB Simulation & Processing

    This shell file runs successive scripts and gauges progress.

    TODO:
        - Fix beamsteering prediction
        - Separate CPIs
        - Multiple
        - Multistatic processing
        - Save detection coordinates
        - Frequency diversity

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

%% Testing Loop

% spacing_list = 1000*(1:10);
% dist_list = 1000*(1:7);
% spacing_list = 1000*(1:10);
% dist_list = 1000*(1:6);
spacing_list = 7000;
dist_list = 5000;

num_iter = 25;
num_frames = 500;

% mean_out = nan(2, length(sigma_list), length(sigma_list), num_iter);
error_out = nan(3, num_frames, length(spacing_list), length(dist_list), num_iter);
est_out = nan(2, num_frames, length(spacing_list), length(dist_list), num_iter);
traj_out = est_out;
single_out = est_out;
mean_out = nan(3, length(spacing_list), length(dist_list), num_iter);
std_out = mean_out;
rms_out = mean_out;
max_out = mean_out;
num_miss = nan(length(spacing_list), length(dist_list), num_iter);
num_fail = num_miss;

tic;

sig_x_in = 0.09;
sig_y_in = 0;

for dist_ind = 1:length(dist_list)
    for spacing_ind = 1:length(spacing_list)
        
        fprintf('Starting Monte Carlo for distance %d, spacing %d\n', dist_list(dist_ind), spacing_list(spacing_ind));
        
        dist_in = dist_list(dist_ind);
        spacing_in = spacing_list(spacing_ind);
%         
%         max_vel = 250;
%         frame_time = 0.0512;
%         dist_per_frame = max_vel * frame_time;
%         frames_in = ceil(2 * spacing_in / dist_per_frame);
        frames_in = num_frames;
        
%         fprintf('Simulating %d frames\n', frames_in);
        
        for iter = 1:num_iter
            
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
            Main_NoSimulation
            
            %% TEMP:PLOTS
            
            MultiPlot_Scratch
            
            %             mean_out(:,x_ind,y_ind,iter) = mean(error_diff, 2, 'omitnan');
            %             std_out(:,x_ind,y_ind,iter) = std(error_diff, [], 2, 'omitnan');
            traj_out(:,:,spacing_ind,dist_ind,iter) = traj(1:2,:);
            est_out(:,:,spacing_ind,dist_ind,iter) = track_multi;
            single_out(:,:,spacing_ind,dist_ind,iter) = single;
            error_out(:,:,spacing_ind,dist_ind,iter) = error_diff;
            mean_out(:,spacing_ind,dist_ind,iter) = mean(error_diff, 2, 'omitnan');
            std_out(:,spacing_ind,dist_ind,iter) = std(error_diff, [], 2, 'omitnan');
            rms_out(:,spacing_ind,dist_ind,iter) = rms(error_diff', 'omitnan')';
            max_out(:,spacing_ind,dist_ind,iter) = max(error_diff, [], 2, 'omitnan');
            num_miss(spacing_ind,dist_ind,iter) = sum(isnan(error_diff(1,:)));
            num_fail(spacing_ind,dist_ind,iter) = sum(error_diff(3,:) > 10);
            
        end
        
        toc;
    end
end

MultiPlot_Scratch2

% save('Varied.mat', 'traj_out', 'est_out', 'single_out', 'error_out', 'mean_out', 'std_out', 'rms_out', 'max_out', 'num_miss', 'num_fail', 'spacing_list', 'dist_list');

%% Save Figures and Data

% Run all end-of-simulation tasks
EndProcess




