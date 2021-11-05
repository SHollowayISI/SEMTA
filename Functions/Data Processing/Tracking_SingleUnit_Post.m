function [track_out] = Tracking_SingleUnit_Post(scenario, unit, passDirection)
%TRACKING_SINGLEUNIT Performs radar target tracking for single-unit case
%   Takes radar scenario object as input, returns modified scenario.multi
%   as output.

%% Unpack Variables

track_in = scenario.tracking_single{unit};
multi = scenario.multi;
rs = scenario.radarsetup;
ts = rs.tracking_single;

n_fr = multi.num_fr(unit);

%% Initialize output object

track_out = struct( ...
    'isActive',     false, ...
    'misses',       0, ...
    'hit_list',     track_in.hit_list, ...
    'meas',         {track_in.meas}, ...
    'estimate',     {cell(n_fr, 1)}, ...
    'prediction',   {cell(n_fr, 1)});

%% Target tracking

% Determine pass direction
if strcmp(passDirection, 'forward')
    fr_ind = 1:n_fr;
elseif strcmp(passDirection, 'reverse')
    fr_ind = n_fr:-1:1;
end

% Loop through frames
for fr = fr_ind
    
    % Check if detection occured
    if track_out.hit_list(fr)
            
        % Set flags/variables
        track_out.isActive = true;
        track_out.misses = 0;
    else
        
        % Set flags/variables
        track_out.misses = track_out.misses + 1;
        track_out.isActive = track_out.isActive && (track_out.misses < ts.miss_max);
    end
    
    % Perform track filtering
    if track_out.isActive
        track_out = KalmanFilter_SingleUnit(track_out, rs, ts, fr, multi.time{unit}(fr), passDirection);
    end
    
end

end

