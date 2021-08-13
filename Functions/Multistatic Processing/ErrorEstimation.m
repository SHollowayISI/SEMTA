function [results] = ErrorEstimation(scenario)
%ERRORESTIMATION_SEMTA Estimate errors from SEMTA simulations
%   Takes radar scenario object as input, returns modified scenario.results
%   object as output.

%% Unpack Variables

traj = scenario.traj;
multi = scenario.multi;
radarsetup = scenario.radarsetup;

%% Unpack trajectory positions

% Calculate points in trajectory
traj_ind = round(radarsetup.cpi_fr * radarsetup.n_p * ...
    (0:(multi.n_fr-1))+1);
traj_points = traj.pos(:,traj_ind);

% Determine true range and angle from each receiver
true_ranges = zeros(length(traj_ind), multi.n_re);
for fr = 1:length(traj_ind)
    for re = 1:multi.n_re
        true_ranges(fr, re) = rangeangle(traj_points(:,fr), multi.radar_pos(:,re));
    end
end

%% Calculate monostatic errors

% Determine range error for each frame
results.range_error = true_ranges - multi.ranges;
results.range_error_rms = squeeze(rms(results.range_error, 1));

%% Calculate multistatic errors

% Determine trilateration error for each frame
results.trilat_error = multi.lat_points - traj_points(1:2,:);
results.trilat_error_rms = rms(results.trilat_error, 2);

% Determine trilateration error with tracking
results.track_error = multi.track_points - traj_points(1:2,:);
results.track_error_rms = rms(results.track_error, 2);


end

