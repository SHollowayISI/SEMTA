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
combined_hit_list = track_forward.hit_list_out & track_reverse.hit_list_out;

%% Perform data fusion

for fr = 1:scenario.multi.n_fr
    
    % Unpack structures
    est_f = track_forward.track_estimate{fr};
    est_r = track_reverse.track_estimate{fr};
    
    if combined_hit_list(fr)
        
        % Perform inverse variance weighting
        var_sum = (1 ./ diag(est_f.covar)) + (1 ./ diag(est_r.covar));
        state_sum = (est_f.state ./ diag(est_f.covar)) + (est_r.state ./ diag(est_r.covar));
        var_new = diag(1 ./ var_sum);
        state_new = state_sum ./ var_sum;
        
        % Save data
        tracking_multi.track_estimate{fr}.state = state_new;
        tracking_multi.track_estimate{fr}.covar = var_new;
        tracking_multi.track_estimate{fr}.pos = state_new([1 4]);
        tracking_multi.track_estimate{fr}.vel = state_new([2 5]);
        
    elseif track_forward.hit_list_out(fr)
        
        % Save only forward results
        tracking_multi.track_estimate{fr}.state = est_f.state;
        tracking_multi.track_estimate{fr}.covar = est_f.covar;
        tracking_multi.track_estimate{fr}.pos = est_f.state([1 4]);
        tracking_multi.track_estimate{fr}.vel = est_f.state([2 5]);
        
    elseif track_reverse.hit_list_out(fr)
        
        % Save only forward results
        tracking_multi.track_estimate{fr}.state = est_r.state;
        tracking_multi.track_estimate{fr}.covar = est_r.covar;
        tracking_multi.track_estimate{fr}.pos = est_r.state([1 4]);
        tracking_multi.track_estimate{fr}.vel = est_r.state([2 5]);
        
    end
    
    est_f.pos
    est_r.pos
    tracking_multi.track_estimate{fr}.pos
    
end

%% Generate new detection list

tracking_multi.bi_hit_list = track_forward.hit_list_out | track_reverse.hit_list_out;

end

