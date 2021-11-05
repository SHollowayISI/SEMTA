function [] = SaveTracking(scenario)
%SAVETRACKING Saves tracking data to Python-compatible format
%   Takes scenario as input, flattens to array data structure
%
%   NOTE: DEPRECATED AS OF ASYNCHRONOUS BRANCH

%% Unpack variables

n_re = scenario.multi.n_re;
n_fr = scenario.multi.n_fr;
track_in = scenario.tracking_single;

%% Create top level structure

track_out = struct( ...
    'hit_list',     false(n_re, n_fr), ...
    'range',        nan(n_re, n_fr), ...
    'vel',          nan(n_re, n_fr), ...
    'SNR',          nan(n_re, n_fr), ...
    'az',           nan(n_re, n_fr), ...
    'steer',        nan(n_re, n_fr), ...
    'radar_pos',    nan(3, n_re)); 

%% Loop through frames/receivers

for re = 1:n_re
    
    track_out.radar_pos(:,re) = scenario.multi.radar_pos(:,re);
    
    for fr = 1:n_fr
        
        track_out.hit_list(re,fr) = track_in{re}.hit_list(fr);
        
        if track_out.hit_list(re, fr)
            track_out.range(re,fr) = track_in{re}.meas{fr}.range(1);
            track_out.vel(re,fr) = track_in{re}.meas{fr}.vel(1);
            track_out.SNR(re,fr) = track_in{re}.meas{fr}.SNR(1);
            track_out.az(re,fr) = track_in{re}.meas{fr}.az(1);
            track_out.steer(re,fr) = track_in{re}.meas{fr}.steer(1);
        end
        
    end
end

%% Save to file

filename = ['Results/Tracking/', scenario.simsetup.filename, '_', datestr(now, 'mmddyy_HHMM'), '.mat'];
save(filename, 'track_out');

disp(['Tracking results saved in ', filename]);

end

