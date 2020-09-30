%% SEMTA Radar System - Main Simulation Loop
%{

    Sean Holloway
    SEMTA Main Simulation Loop
    
    This file specifies performs simulation, signal processing, detection,
    data processing, multistatic processing, and results collection for
    SEMTA system. 

    Use script 'FullSystem_AutomatedSimulation.m' to run scenarios.
    
%}

%% Main Loop

% Start timing for estimation
timeStart(scenario);

% Loop through radar units
for unit = 1:scenario.multi.n_re
    
    % Set current unit flag
    scenario.flags.unit = unit;
    
    
    % Loop through simulation frames
    for frame = 1:scenario.multi.n_fr
        
        % Set current frame flag
        scenario.flags.frame = frame;
        
        
        %% Radar Simulation (Single frame)
        
        % Run simulation to retrieve fast time x slow time Rx signal
        scenario = RadarSimulation_SEMTA(scenario);
        
        
        %% Signal Processing (Single frame)
        
        % Perform signal processing on received signal
        scenario.cube = SignalProcessing_SEMTA(scenario);
        
        
        %% Data Processing (Single frame)
        
        % Perform radar detection
        scenario.detection = Detection_SEMTA(scenario);
        
        
        %% Save Multistatic Information
        
        % Read out target information
        readOut(scenario);
        
        % Store and clear target information
        storeMulti(scenario);
        
        % Read out current progress in simulation
        frameUpdate(scenario, 1);
        
        % Estimate remaining time in simulation
        timeUpdate(scenario, 5, 'frames');
        
    end
    
    % Read out current proress in simulation
    unitUpdate(scenario, 1);
    
end

%% Visualize Monostatic Results

% View Range-Doppler heat map
% viewRDCube(scenario, 'heatmap')

% View Range-Doppler surface
% viewRDCube(scenario, 'surface')

% View radar detections
% viewDetections(scenario);

% View SNR of each radar unit
viewSNR(scenario);


%% Multistatic Processing

% Perform multilateration on list of detected targets
scenario.multi = MultistaticProcessing_SEMTA(scenario);

% Perform smoothing on multilaterated target coordinates
scenario.multi = TargetTracking_SEMTA(scenario);

% Visualize multilateration result
viewMultilateration(scenario);

%% Results Processing

% Estimate error of results
scenario.results = ErrorEstimation_SEMTA(scenario);

% View plots of errors
viewErrors(scenario);

