function [scenario] = Tracking_SingleUnit(scenario)
%TRACKING_SINGLEUNIT Performs radar target tracking for single-unit case
%   Takes radar scenario object as input, returns modified scenario.multi
%   as output.

%% Unpack Variables

% Flags
frame = scenario.flags.frame;
unit = scenario.flags.unit;

% Structs
multi = scenario.multi;
det_logical = scenario.detection.detect_logical;
det_list = scenario.detection.detect_list;
track = scenario.tracking_single{unit};

% Parameters
rs = scenario.radarsetup;
ts = rs.tracking_single;

%% Initialize Track Object

% Initialize tracking data structure on first frame
if frame == 1
    track = struct( ...
        'isActive',     false, ...                  % Is track still alive
        'misses',       0, ...                      % Number of missed detections
        'hit_list',     false(multi.n_fr, 1), ...   % Frames in which a detection was added
        'meas',         {cell(multi.n_fr, 1)}, ...    % List of radar measurements
        'estimate',     {cell(multi.n_fr, 1)}, ...    % List of tracking estimates
        'prediction',   {cell(multi.n_fr, 1)});       % List of tracking predictions
end

%% Target-to-Track Association

% Check if detection occured
if det_logical
    
    % If track is currently active
    if track.isActive
        
        % Loop through detections and determine if any is to be added
        [hitFound, meas] = targetToTrack(track, det_list);
        
        % If detection is added to track
        if hitFound
            % Add measurement
            track.meas{frame} = meas;
            track.hit_list(frame) = true;
            track.misses = 0;
        else
            % Add miss
            track = addMiss(track);
        end
    else
        
        % Add strongest detection to track
        track.meas{frame} = targetToNewTrack(det_list); 
        track.isActive = true;
        track.hit_list(frame) = true;
    end
else
    
    % If no detections occured, increase misses
    track = addMiss(track);
end

    
%% Track Filtering

% Update Kalman tracking filter
if track.isActive
    track = KalmanFilter_SingleUnit(track, rs, ts, frame);
end

%% Pack Variables

scenario.tracking_single{unit} = track;

%% Target-To-Track Function Declarations

% Add miss to track
function [track] = addMiss(track)
    
    % Increase misses
    track.misses = track.misses + 1;
    
    % Inactivate track if misses exceed maximum
    if track.isActive && (track.misses > ts.miss_max)
        track.isActive = false;
    end
end

% Convert detection list index to measurement structure
function [meas] = detectionListToMeasurement(det_list, index)
    
    % Create measurement structure
    meas = struct( ...
        'range',    det_list.range(index), ...
        'vel',      det_list.vel(index), ...
        'az',       det_list.az(index), ...
        'cart',     det_list.cart(:,index), ...
        'SNR',      det_list.SNR(index), ...
        'steer',    multi.steering_angle(frame, unit));
end

% Target-to-track association
function [hitFound, meas] = targetToTrack(track, det_list)
    
    % T/F are detections still possible
    isValid = true(det_list.num_detect, 1);
    
    % Coarse gating
    delta_range = abs(det_list.range - track.estimate{frame-1}.range);
    max_delta = rs.frame_time * Inf *  ts.max_vel;
    isValid = isValid & (delta_range < max_delta);
    
    % Fine gating using statistically weighted distance
    dist = MahanalobisDistance(track, det_list, rs, ts, frame);
    isValid = isValid & (dist < ts.dist_thresh)';
    
    % Check if any detections are valid
    if ~any(isValid)
        hitFound = false;
        meas = [];
        return
    end
    
    % Return most likely detection
    hitFound = true;
    [~, min_ind] = min(dist);
    meas = detectionListToMeasurement(det_list, min_ind);
    
end

% Strongest target calculation for new track association
function [meas] = targetToNewTrack(det_list)
    
    % Find index of strongest SNR detection
    [~, max_ind] = max(det_list.SNR);
    meas = detectionListToMeasurement(det_list, max_ind);
end

% Range prediction for current frame
function [nextRange] = predictRange(track)
    
    % Calculate next range prediction
    Tm = rs.frame_time;
    X = track.estimate{frame-1}.state;
    nextRange = sqrt((X(1) + Tm*X(2))^2 + (X(3) + Tm*X(4))^2);
end

end

