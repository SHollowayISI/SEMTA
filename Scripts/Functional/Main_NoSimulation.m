%% SEMTA Radar System - Main Simulation Loop
%{

    Sean Holloway
    SEMTA Main Simulation Loop
    
    This file specifies performs simulation, signal processing, detection,
    data processing, multistatic processing, and results collection for
    SEMTA system. 

    Use script 'FullSystem.m' to run scenarios.
    
%}

%% Main Loop

% Start timing for estimation
timeStart(scenario);

meas_single = nan(2,scenario.multi.n_fr,scenario.multi.n_re);
track_single = meas_single;
track_single_pre = meas_single;
% 
% xyvar = nan(4, scenario.multi.n_fr, scenario.multi.n_re);

% fom_limit = nan(scenario.multi.n_fr, scenario.multi.n_re);

% Loop through radar units
for unit = 1:scenario.multi.n_re
    
    % Set current unit flag
    scenario.flags.unit = unit;
    
    % Loop through simulation frames
    for frame = 1:scenario.multi.n_fr
        
        % Set current frame flag
        scenario.flags.frame = frame;
        
        %% Beam Steering Setup
        
        % Set beam steering direction
        scenario.multi.steering_angle(frame, unit) = BeamsteeringUpdate(scenario, false);
        
        %% Generate Simulated Data
        
        % Generate resolutions and axes
        scenario.cube = SimulateAxes(scenario);
        
        % Randomly generate simulation values
        scenario = SimulateDetections(scenario, 10.5);
        
        %% Data Processing (Single frame)
        
        % Perform single unit tracking
        scenario = Tracking_SingleUnit(scenario);
        
        % Store and clear target information
        storeMulti(scenario);
        
        % Check for mode changes
        scenario = ModeCheck(scenario, false);
        
    end
    
    % Reset parameters for new unit
    unitReset(scenario);
    
    % Read out current proress in simulation
    % unitUpdate(scenario, 1);

    %% Visualize Monostatic Results

    % View Range-Slow Time heat map
    % viewRangeCube(scenario);

    % View Range-Doppler heat map
    % viewRDCube(scenario);

    % View radar detections
    % viewDetections(scenario);

    % View tracking results
    % viewTrackingSingle(scenario);

    % View SNR of each radar unit
    % viewSNR(scenario);
    
end


%% Multistatic Processing

% Fuse data between multiple estimations
scenario.tracking_multi = DataFusion(scenario);

% Perform Kalman filter tracking on fused sensor data
if strcmp(scenario.radarsetup.tracking_single.track_dir, 'combined')
    scenario.tracking_multi = Tracking_Multi_Bidirectional(scenario);
else
    scenario.tracking_multi = Tracking_Multi(scenario, scenario.radarsetup.tracking_single.track_dir);
end

% Visualize multilateration result
% viewMultilateration(scenario);




