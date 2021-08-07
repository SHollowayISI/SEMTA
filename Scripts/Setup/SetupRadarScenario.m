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
    'tx_pow',       4, ...              % Transmit power in Watts (per channel?)
    'tx_ant_gain',  27, ...             % Tx antenna gain in dBi (N/A?)
    'rx_sys_gain',  0, ...              % Rx system gain in dB (N/A?)
    'rx_nf',        4, ...              % Rx noise figure in dB
    'rx_ant_gain',  27, ...             % Rx antenna gain in dBi (N/A?)
    ...
    ... % Processing Properties
    'win_type',     'blackmanharris', ...   % Type of window for doppler processing
    ...
    ... % Detection Properties
    'Pfa',          1e-6, ...           % Probability of false alarm for CFAR
    'det_m',        2);                 % M for m-of-n processing


%% Run Setup Scripts

% Set up Phased Array Toolbox system objects
scenario.sim = PhasedSetup(scenario);





