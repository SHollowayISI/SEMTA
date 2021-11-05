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
startProgressTimer(scenario);

% Loop through radar units
for unit = 1:scenario.multi.n_re
    
    % Set current unit flag
    scenario.flags.unit = unit;
    
    % Set simulation times
    simTimeStart(scenario);
    
    % Loop until simulation time is surpassed
    while scenario.flags.frameStartTime < scenario.multi.sim_time
        
        % Set current frame flag
        scenario.flags.frame = scenario.flags.frame + 1;
        
        %% Beam Steering Setup
        
        % Set beam steering direction
        scenario.multi.steering_angle{unit}(end+1) = BeamsteeringUpdate(scenario, true);
        
        %% Radar Simulation (Single frame)
        
        % Run simulation to retrieve fast time x slow time Rx signal
        scenario = RadarSimulation(scenario);
        
        %% Signal Processing (Single frame)
        
        % Perform signal processing on received signal
        scenario.cube = SignalProcessing(scenario);
        
        %% Data Processing (Single frame)
        
        % Perform radar detection
        scenario.detection = Detection(scenario);
        
        % Perform single unit tracking
        scenario = Tracking_SingleUnit(scenario);
        
        % Store and clear target information
        storeMulti(scenario);
        
        % Check for mode changes
        scenario = ModeCheck(scenario, true);
        
        % Update timing information
        simTimeUpdate(scenario);
        
        %% Read Out Progress
        
        % Read out target information
        readOut(scenario);
        
        % Read out current progress in simulation
        frameUpdate(scenario, 1);
        
        % Estimate remaining time in simulation
        updateProgressTimer(scenario, 5, 'frames');
        
    end
    
    % Save single unit tracking
    if scenario.simsetup.save_track_single
        SaveTrackingSingle(scenario, unit);
    end
    
    % Post-process tracking
    if scenario.radarsetup.tracking_single.bi_single
        scenario = Tracking_SingleUnit_Bidirectional(scenario);
    end
    
    % Reset parameters for new unit
    unitReset(scenario);
    
    % Read out current proress in simulation
    unitUpdate(scenario, 1);
    
end

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


%% Multistatic Processing

% Fuse data between multiple estimations
scenario.tracking_multi = DataFusion(scenario);

% Perform Kalman filter tracking on fused sensor data
if scenario.radarsetup.tracking_single.bi_multi
    scenario.tracking_multi = Tracking_Multi_Bidirectional(scenario);
else
    scenario.tracking_multi = Tracking_Multi(scenario, 'forward');
end

% Visualize multilateration result
viewTrackingMulti(scenario);

%% Results Processing

% Estimate error of results
% scenario.results = ErrorEstimation(scenario);

% View plots of errors
% viewErrors(scenario);

