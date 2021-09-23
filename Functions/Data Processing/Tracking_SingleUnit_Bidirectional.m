function [scenario] = Tracking_SingleUnit_Bidirectional(scenario)
%TRACKING_SINGLEUNIT_BIDIRECTIONAL Perform two-pass tracking between frames
%of single sensor data
%   Takes scenario object as input and returns smoothed list of track
%   estimates.

%% Unpack variables

unit = scenario.flags.unit;
track = scenario.tracking_single{unit};

% Prepare new variables
track.estimate = cell(scenario.multi.n_fr, 1);

%% Run each pass of tracking

track_forward = Tracking_SingleUnit_Post(scenario, unit, 'forward');
track_reverse = Tracking_SingleUnit_Post(scenario, unit, 'reverse');
hit_list = track_forward.hit_list & track_reverse.hit_list;

%% Perform data fusion

for fr = 1:scenario.multi.n_fr
    
    if hit_list(fr)
        
        % Unpack structures
        est_f = track_forward.estimate{fr};
        est_r = track_reverse.estimate{fr};
        
        % Perform inverse variance weighting
        var_sum = (1 ./ diag(est_f.covar)) + (1 ./ diag(est_r.covar));
        state_sum = (est_f.state ./ diag(est_f.covar)) + (est_r.state ./ diag(est_r.covar));
        var_new = diag(1 ./ var_sum);
        state_new = state_sum ./ var_sum;
        
        % Save data
        track.estimate{fr}.state = state_new;
        track.estimate{fr}.covar = var_new;
        track.estimate{fr}.cart = state_new([1 3]);
        track.estimate{fr}.az = atand(state_new(3) / state_new(1));
        track.estimate{fr}.range = sqrt(sum(state_new([1 3]).^2));
        track.estimate{fr}.speed = sqrt(sum(state_new([2 4]).^2));
        
    end
    
end

%% Pack variables

scenario.tracking_single{unit} = track;

end

