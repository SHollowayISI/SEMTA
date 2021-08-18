function [sig_R, sig_A, sig_V] = CalculateVariance(meas)
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

%% Calculate variances

% Variance in range
if range < 1500
    sig_R = 2.3893 - 2.1567 * (range / 1500);
else
    sig_R = 10 ^ (max(0.67736 - 0.041748 * SNR, -0.75438 + 0.083800 * SNR));
end

% Variance in angle
offset = abs(theta - steer);
sig_A_direct = 10 ^ (0.093023 - 0.052088 * SNR);
sig_A_offset = sig_A_direct * sqrt(1.0122 - (0.52055 * offset) + (0.36508 * offset^2));
sig_A = deg2rad(sig_A_offset);

% Variance in velocity
sig_V = 10 ^ (-0.90783 - 0.050062 * SNR);

end

