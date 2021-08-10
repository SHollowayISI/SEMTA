%% SEMTA Radar System - Example Radar Initialization File
%{

    Sean Holloway
    SEMTA Radar Init File
    
    This file specifies radar parameters for SEMTA simulation.

    Use script 'FullSystem.m' to run scenarios.
    
%}

%% Radar Parameter Setup

% Radar simulation and processing setup
scenario.radarsetup = struct( ...
    ...
    ... % Waveform Properties
    'f_c',      9.45e9, ...             % Operating frequency in Hz
    'f_s',      25e6, ...               % ADC sample frequency in Hz
    't_p',      10e-6, ...              % Chirp duration in seconds
    'bw',       10e6, ...               % Bandwidth of chirp
    'prf',      20e3, ...               % Pulse repetition frequency in Hz
    'n_p',      1024, ...               % Number of pulses to simulate
    'cpi_fr',   5, ...                  % Number of CPI per frame
    ...
    ... % Transceiver Properties
    'n_ant',        16, ...             % Number of elements in antenna array
    'tx_pow',       4, ...              % Transmit power in Watts per channel
    'rx_sys_gain',  -1.953, ...         % Rx system gain in dB 
    'rx_ant_gain',  27, ...             % Rx antenna gain in dB 
    'tx_sys_gain',  -1.953, ...         % Tx system gain in dB 
    'tx_ant_gain',  27, ...             % Tx antenna gain in dB 
    'rx_nf',        4, ...              % Rx noise figure in dB
    'beamwidth',    7, ...              % Antenna beamwidth in degrees
    'mono_coeff',   -1.8921, ...        % Coefficient in monopulse AoA linear approximation
    ...
    ... % Processing Properties
    'win_type',     'blackmanharris', ...   % Type of window for doppler processing
    ...
    ... % Detection Properties
    'detect_type',  'CFAR', ...         % Choose 'CFAR' or 'threshold'
    'thresh',       [], ...             % Threshold in dB for threshold detection
    'Pfa',          1e-6, ...           % Probability of false alarm for CFAR
    'num_guard',    [3 3], ...          % Number of R-D guard cells for CFAR detection
    'num_train',    [15 2], ...         % Number of R-D training cells for CFAR detection
    'det_m',        2);                 % M for m-of-n processing

% Calculate derived parameters
scenario.radarsetup.pri = 1/scenario.radarsetup.prf;
scenario.radarsetup.frame_time = ...
    scenario.radarsetup.pri * scenario.radarsetup.n_p * scenario.radarsetup.cpi_fr;


%% Run Setup Scripts

% Set up Phased Array Toolbox system objects
scenario.sim = PhasedSetup(scenario);





