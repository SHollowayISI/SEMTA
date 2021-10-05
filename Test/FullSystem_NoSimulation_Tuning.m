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

sig_x_list = 6:10;
sig_y_list = 0.5;
num_iter = 25;
num_frames = 500;

% mean_out = nan(2, length(sigma_list), length(sigma_list), num_iter);
error_out = nan(3, num_frames, length(sig_x_list), length(sig_y_list), num_iter);
mean_out = nan(3, length(sig_x_list), length(sig_y_list), num_iter);
std_out = mean_out;
rms_out = mean_out;
max_out = mean_out;
num_miss = nan(length(sig_x_list), length(sig_y_list), num_iter);
num_fail = num_miss;

tic;

for x_ind = 1:length(sig_x_list)
    for y_ind = 1:length(sig_y_list)
        
        fprintf('Starting Monte Carlo for sigma of <%d, %d>\n', sig_x_list(x_ind), sig_y_list(y_ind));
        
        dist_in = 5000;
        spacing_in = 3000;
%         
%         max_vel = 250;
%         frame_time = 0.0512;
%         dist_per_frame = max_vel * frame_time;
%         frames_in = ceil(2 * spacing_in / dist_per_frame);
        frames_in = num_frames;
        
        sig_x_in = sig_x_list(x_ind);
        sig_y_in = sig_y_list(y_ind);
        
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
            error_out(:,:,x_ind,y_ind,iter) = error_diff;
            mean_out(:,x_ind,y_ind,iter) = mean(error_diff, 2, 'omitnan');
            std_out(:,x_ind,y_ind,iter) = std(error_diff, [], 2, 'omitnan');
            rms_out(:,x_ind,y_ind,iter) = rms(error_diff', 'omitnan')';
            max_out(:,x_ind,y_ind,iter) = max(error_diff, [], 2, 'omitnan');
            num_miss(x_ind,y_ind,iter) = sum(isnan(error_diff(1,:)));
            num_fail(x_ind,y_ind,iter) = sum(error_diff(3,:) > 10);
            
        end
        
        toc;
    end
end

% save('MultiErrorTest.mat', 'error_out', 'mean_out', 'std_out', 'rms_out', 'max_out', 'num_miss', 'spacing_list', 'dist_list');

%% Save Figures and Data

% Run all end-of-simulation tasks
EndProcess




