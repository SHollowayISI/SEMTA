function [scenario] = SimulateDetections(scenario, SNR_min)
%SIMULATEDETECTIONS Generates radar detections using empirical variances
%   Takes scenario as input and returns modified as output

%% Unpack Variables

rs = scenario.radarsetup;
traj = scenario.traj;
rcs = scenario.rcs;
flags = scenario.flags;
multi = scenario.multi;

% Set up detection variable
detection = struct();

%% Calculate True Coordinates

% Calculate true position and velocity
time = (flags.frame - 0.5)*rs.frame_time;
pos = traj.pos(traj, time);
vel = traj.vel(traj, time);

% Calculate true derived variables
[rng_true, ang_true] = rangeangle(pos, multi.radar_pos(:, flags.unit));
ang_true = ang_true(1);
dop_true = sum([cosd(ang_true); sind(ang_true); 0] .* vel);

% Calculate observed RCS and SNR
RCS = rcs.evaluate(rcs, ang_true, rs.f_c);
SNR = CalculateSNR(scenario, true, true, RCS, rng_true);

% Check if SNR is high enough for detection
if(SNR < SNR_min)
    detection.detect_logical = false;
    detection.detect_list = [];
    scenario.detection = detection;
    return;
else
    detection.detect_logical = true;
end

% Calculate variances
meas = struct( ...
    'steer',    ang_true, ...
    'range',    rng_true, ...
    'az',       ang_true, ...
    'SNR',      SNR);
[sig_R, sig_A, sig_V] = CalculateVariance(meas, rs);

%% Generate Stochastic Results

% Generate using normal distribution
rng_meas = rng_true + sig_R * randn(1);
ang_meas = ang_true + sig_A * randn(1);
dop_meas = dop_true + sig_V * randn(1);

% Generate derived values
cart_meas = rng_meas .* [cosd(ang_meas); sind(ang_meas)];

% Save measurements
detection.detect_list = struct( ...
    'range',        rng_meas, ...
    'vel',          dop_meas, ...
    'az',           ang_meas, ...
    'cart',         cart_meas, ...
    'SNR',          SNR, ...
    'num_detect',   1);

%% Pack Variables

scenario.detection = detection;

end

