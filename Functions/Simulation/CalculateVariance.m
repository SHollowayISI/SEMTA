function [sig_R, sig_A, sig_V] = CalculateVariance(meas, rs)
%CALCULATEVARIANCE Calculates standard deviation related to a measurement
%at a set coordinate and signal-to-noise ratio.
%   Takes measurement data structure as input, returns deviation in range,
%   angle, and velocity. NOTE: I incorrectly used "variance" but these are
%   standard deviations. I will change at some point, I hope.

%% Unpack variables

steer = meas.steer;
range = meas.range;
theta = meas.az;
SNR = meas.SNR;

%% Calculate range variance

% Calculate fading factor due to incomplete pulse return
fade = min([1; ...
    2 * range / (physconst('Lightspeed') * rs.t_p); ...
    (rs.pri - (2 * range / physconst('Lightspeed'))) / rs.t_p]);

% Calculate variance accounting only for SNR
sig_R = 7.8927 ./ sqrt(db2pow(SNR));

% Incorporate fading resolution loss
sig_R = sig_R / fade^2;

%% Calculate angle variance

% Calculate variance accounting only for SNR
sig_A = (rs.beamwidth / abs(rs.mono_coeff)) ./ sqrt(2 * db2pow(SNR));

% Adjust variance for beam steering loss
sig_A = sig_A ./ cosd(steer).^2;

% Adjust variance for offset from beam center
offset = abs(theta - steer);
sig_A = sig_A .* sqrt(1 + (rs.mono_coeff * offset / rs.beamwidth)^2);

%% Calculate velocity variance

% Variance in velocity
sig_V = 10 .^ (-1.2517 - 0.0044602 .* SNR);

end

