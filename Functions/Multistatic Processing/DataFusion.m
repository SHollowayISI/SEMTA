function [tracking_multi] = DataFusion(scenario)
%DATAFUSION Combination of data between multiple radar sensors
%   Performs inverse-variance weighted average of sensor positions and
%   velocity

%% Unpack variables

ts = scenario.tracking_single;
rs = scenario.radarsetup;
multi = scenario.multi;

% Set up data structure
tracking_multi = struct( ...
    'state',        [], ...
    'var',          [], ...
    'time',         []);

%% Sort tracking data in time

% Aggregate list of times and receiver indices
out_list = nan(0,3);
for unit = 1:multi.n_re
    
    % Unpack time stamps
    time_list = multi.time{unit};
    
    % Append time stamp, frame index, and unit index
    for fr = 1:length(time_list)
        if ts{unit}.hit_list(fr)
            out_data = [time_list(fr), fr, unit];
            out_list = [out_list; out_data];
        end
    end
end

% Generate sorted list
detection_list = sortrows(out_list);
n_fr = size(detection_list, 1);

% Add data structures
tracking_multi.state = cell(n_fr, 1);
tracking_multi.var = cell(n_fr, 1);
tracking_multi.time = nan(n_fr, 1);
tracking_multi.num_fr = n_fr;

%% Fuse list of detections

% Loop through detection list
for de = 1:n_fr
    
    % Get unit number and single unit frame number
    re = detection_list(de,3);
    fr = detection_list(de,2);
    
    % Adjust for radar position
    offset = [multi.radar_pos(1,re); 0; 0; multi.radar_pos(2,re); 0; 0];
    
    % Pass data to combined list
    tracking_multi.state{de} = ts{re}.estimate{fr}.state + offset;
    tracking_multi.var{de} = ts{re}.estimate{fr}.covar;
    tracking_multi.time(de) = ts{re}.estimate{fr}.time;
        
end

end

