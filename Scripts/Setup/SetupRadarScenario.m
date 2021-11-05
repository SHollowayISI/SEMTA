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
    ...
    ... % Transceiver Properties
    'n_ant',        16, ...             % Number of elements in antenna array
    'tx_pow',       4, ...              % Transmit power in Watts per channel
    'rx_ant_gain',  27, ...             % Rx antenna gain in dB 
    'tx_ant_gain',  27, ...             % Tx antenna gain in dB 
    'rx_nf',        4, ...              % Rx noise figure in dB
    'range_off',    true, ...           % Correct range with offset function
    'beamwidth',    6.335, ...          % Antenna beamwidth in degrees
    'mono_coeff',   -1.4817, ...        % Coefficient in monopulse AoA linear approximation
    'phase_bits',   6, ...              % Number of bits for phase shifter resolution Nbits = log2(360/resolution)
    ...
    ... % Processing Properties
    'r_win',        'hamming', ...      % Type of window for range processing
    'd_win',        'none', ...         % Type of window for doppler processing
    ...
    ... % Detection Properties
    'detect_type',  'CFAR', ...         % Choose 'CFAR' or 'threshold'
    'thresh',       [], ...             % Threshold in dB for threshold detection
    'Pfa',          1e-6, ...           % Probability of false alarm for CFAR
    'num_guard',    [3 3], ...          % Number of R-D guard cells for CFAR detection
    'num_train',    [15 2], ...         % Number of R-D training cells for CFAR detection
    'rng_limits',   [500, 7000], ...    % Min/max range values, to avoid false alarms
    'vel_comp',     true, ...           % T/F compensate for range bin migration in binary integration
    'det_m',        2);                 % M for m-of-n processing

% Tracking Parameters
tracking = struct( ...
    'max_vel',           250, ...            % Maximum possible speed for coarse gating
    'max_acc',           1, ...              % Maximum possible acceleration for uncertainty estimation
    'dist_thresh',       Inf, ...13.8, ...           % Mahanalobis distance threshold for fine gating
    'miss_max',          5, ...              % Number of misses required to inactivate track
    'EKF',               false, ...          % T/F use extended Kalman filter
    'sigma_v',           [0.09, 0], ...       % XY target motion uncertainty 
    'sigma_v_multi',     [0.09, 0], ...       % Motion uncertainty for multilaterated tracking
    'bi_multi',          true, ...        % Use bidirectional tracking?
    'bi_single',         true, ...
    'limitSensorFusion', true);         % Only take top two sensor results

scenario.radarsetup.tracking_single = tracking;

%% Radar Mode Setup

% Set initial mode
scenario.radarsetup.initial_mode = 'search';
scenario.flags.mode = scenario.radarsetup.initial_mode;

% Wait mode properties
wait_mode = struct( ...
    'init_angle',   0, ...              % Idle beam steering angle
    'int_type',     'binary', ...       % Integration type for wait mode
    'num_cpi',      5);                 % Number of CPI for integration

% Static mode properties
static_mode = struct( ...
    'init_angle',   0, ...              % Constant beam steering angle
    'int_type',     'incoherent', ...   % Integration type for static mode
    'num_cpi',      1);                 % Number of CPI for integration

% Search mode properties
search_mode = struct( ...
    'init_angle',   45, ...             % Initial angle
    'search_step',  -5, ...             % Angle delta per dwell, in degrees
    'search_max',   45, ...             % Maximum angle for search mode
    'int_type',     'binary', ...       % Integration type for search mode
    'num_cpi',      5);                 % Number of CPI for integration

% Track mode properties
track_mode = struct( ...
    'fallback',     'search', ...       % Fallback mode if detection is lost
    'init_angle',   0, ...              % Initial angle
    'int_type',     'incoherent', ...   % Integration type for track mode
    'num_cpi',      1);                 % Number of CPI for integration

% Ideal (debug) track mode properties
ideal_track_mode = struct( ...
    'init_angle',   0, ...              % Initial angle (unused)
    'int_type',     'incoherent', ...   % Integration type for ideal track mode
    'num_cpi',      1);                 % Number of CPI for integration

% Add to data structure
scenario.radarsetup.modes = struct( ...
    'static',       static_mode, ...
    'wait',         wait_mode, ...
    'search',       search_mode, ...
    'track',        track_mode, ...
    'ideal',        ideal_track_mode);

% Set starting values in data structure
scenario.radarsetup.int_type = ...
    scenario.radarsetup.modes.(scenario.radarsetup.initial_mode).int_type;
scenario.radarsetup.cpi_fr = ...
    scenario.radarsetup.modes.(scenario.radarsetup.initial_mode).num_cpi;

% Calculate derived parameters
scenario.radarsetup.pri = 1/scenario.radarsetup.prf;
scenario.radarsetup.cpi_time = ...
    scenario.radarsetup.pri * scenario.radarsetup.n_p;
scenario.radarsetup.frame_time = ...
    scenario.radarsetup.cpi_time* scenario.radarsetup.cpi_fr;

%% Run Setup Scripts

% Set up Phased Array Toolbox system objects
scenario.sim = PhasedSetup(scenario);





