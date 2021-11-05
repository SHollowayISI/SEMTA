function [SNR] = CalculateSNR(scenario, angleEffects, straddleEffects, RCS, Range)
%CALCULATESNR Calculates SNR of target for SEMTA scenario
%   Takes radar scenario object as input, 
%   provides SNR value as output.

%% Unpack Variables
rs = scenario.radarsetup;

%% Set argument defaults

if ~exist('RCS', 'var')
    RCS = db2pow(scenario.rcs.ave_rcs);
end

if ~exist('Range', 'var')
    current_time = (scenario.flags.frame-1) * scenario.radarsetup.frame_time;
    current_position = scenario.traj.pos(scenario.traj, current_time);
    Range = sqrt(sum((current_position - scenario.multi.radar_pos(:,scenario.flags.unit)).^2));
end

if ~exist('angleEffects', 'var')
    angleEffects = true;
end

if ~exist('straddleEffects', 'var')
    straddleEffects = true;
end

%% Calculate SNR

% Total power = power per channel x number of channels
total_pow = rs.tx_pow * rs.n_ant;

% Wavelength
lambda = physconst('LightSpeed')/rs.f_c;

% Number of pulses to integrate
n_p = rs.n_p;

% Noise figure
NF = db2pow(rs.rx_nf);

% Waveform bandwidth
BW = 1 / rs.t_p;

% Constants
c = (4*pi)^3;
n = physconst('Boltzmann')*290;

% Tx and Rx array gain
Gt = db2pow(rs.tx_ant_gain);
Gr = db2pow(rs.rx_ant_gain);

% Loss due to Rx Steering at 3dB points (removed)
% steering_loss = 0.5;

% Loss due to use of window function
% if strcmp(rs.r_win, 'hamming')
%     processing_loss = db2pow(-1.35);
% else
%     processing_loss = 1;
% end

% Power fading due to transmit and receive overlap
fade = [1; ...
    2 * Range / (physconst('Lightspeed') * rs.t_p); ...
    (rs.pri - (2 * Range / physconst('Lightspeed'))) / rs.t_p];
fading_loss = min(fade.^2);

% Power loss due to steering angle
if angleEffects
    loadIn = load([pwd, '\Results\Antenna Loss\ArrayLoss.mat'], 'angles', 'totalLoss');
    angLossdB = interp1(loadIn.angles, loadIn.totalLoss, abs(scenario.multi.steering_angle{scenario.flags.unit}(scenario.flags.frame)), 'nearest');
    ang_loss = db2pow(-angLossdB);
else
    ang_loss = 1;
end

% Power loss due to straddle loss
if straddleEffects
    offset = 3 * 2 * abs(Range - scenario.cube.range_res*round(Range / scenario.cube.range_res)) / scenario.cube.range_res;
    straddle_loss = db2pow(-offset);
else
    straddle_loss = 1;
end
    
% SNR Calculation
SNR_abs = (total_pow * fading_loss * ang_loss * straddle_loss ...
    * Gt * Gr * n_p * lambda * lambda * RCS) ...
    ./ (c * (Range.^4) * n * NF * BW);

% Return in dB form
SNR = pow2db(SNR_abs);

end

