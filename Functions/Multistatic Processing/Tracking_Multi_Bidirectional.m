function [tracking_multi] = Tracking_Multi_Bidirectional(scenario)
%TRACKING_MULTI_BIDIRECTIONAL Perform two-pass tracking between frames of combined sensor data
%   Takes scenario object as input and returns smoothed list of track
%   estimates.

%% Unpack variables

tracking_multi = scenario.tracking_multi;

% Prepare new variables
tracking_multi.track_estimate = cell(scenario.multi.n_fr, 1);

%% Run each pass of tracking

track_forward = Tracking_Multi(scenario, 'forward');
track_reverse = Tracking_Multi(scenario, 'reverse');

%% Perform data fusion

for fr = 1:scenario.multi.n_fr
    
    % Unpack structures
    est_f = track_forward.track_estimate{fr};
    est_r = track_reverse.track_estimate{fr};
    
    % Perform inverse variance weighting
    var_sum = (1 ./ diag(est_f.covar)) + (1 ./ diag(est_r.covar));
    state_sum = (est_f.state ./ diag(est_f.covar)) + (est_r.state ./ diag(est_r.covar));
    var_new = diag(1 ./ var_sum);
    state_new = state_sum ./ var_sum;
    
    
    
    % Save data
    tracking_multi.track_estimate{fr}.state = state_new;
    tracking_multi.track_estimate{fr}.covar = var_new;
    tracking_multi.track_estimate{fr}.pos = state_new([1 3]);
    tracking_multi.track_estimate{fr}.vel = state_new([2 4]);
    
end

end

