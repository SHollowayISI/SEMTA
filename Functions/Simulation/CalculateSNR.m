function [SNR] = CalculateSNR(scenario, RCS, Range)
%CALCULATESNR Calculates SNR of target for SEMTA scenario
%   Takes radar scenario object as input, 
%   provides SNR value as output.

%% Unpack Variables
radarsetup = scenario.radarsetup;

%% Set argument defaults

if ~exist('RCS', 'var')
    RCS = db2pow(scenario.rcs.ave_rcs);
end

if ~exist('Range', 'var')
    %TODO: UPDATE WITH TRAJECTORY
    Range = sqrt(sum((scenario.traj.pos_st - scenario.multi.radar_pos).^2));
end

%% Calculate SNR

total_pow = radarsetup.tx_pow * radarsetup.n_ant;
lambda = physconst('LightSpeed')/radarsetup.f_c;
n_p = radarsetup.n_p;
NF = db2pow(radarsetup.rx_nf);
c = (4*pi)^3;
n = physconst('Boltzmann')*290;
BW = radarsetup.bw;


Gt = db2pow(radarsetup.tx_ant_gain);
Gr = db2pow(radarsetup.rx_ant_gain);


SNR_abs = (total_pow * Gt * Gr * n_p * lambda * lambda * RCS) ...
    ./ (c * (Range.^4) * n * NF * BW);

SNR = pow2db(SNR_abs);

end

