function [] = SaveTrackingSingle(scenario, unit)
%SAVETRACKINGSINGLE Saves single-unit tracking data to Python-compatible format
%   Takes scenario as input, flattens to array data structure

%% Unpack variables

track_in = scenario.tracking_single{unit};
n_fr = scenario.multi.num_fr(unit);
n_fr_hit = sum(track_in.hit_list);

%% Create top level structure

track_out = struct( ...
    'hit_list',     false(n_fr_hit, 1), ...
    'range',        nan(n_fr_hit, 1), ...
    'vel',          nan(n_fr_hit, 1), ...
    'SNR',          nan(n_fr_hit, 1), ...
    'az',           nan(n_fr_hit, 1), ...
    'steer',        nan(n_fr_hit, 1), ...
    'time',         nan(n_fr_hit, 1), ...
    'radar_pos',    nan(3, 1)); 

%% Loop through frames/receivers

track_out.radar_pos = scenario.multi.radar_pos;
track_out.hit_list = track_in.hit_list;

fr_hit = 1;
for fr = 1:n_fr
    
    if track_out.hit_list(fr_hit)
        track_out.range(fr_hit)     = track_in.meas{fr_hit}.range(1);
        track_out.vel(fr_hit)       = track_in.meas{fr_hit}.vel(1);
        track_out.SNR(fr_hit)       = track_in.meas{fr_hit}.SNR(1);
        track_out.az(fr_hit)        = track_in.meas{fr_hit}.az(1);
        track_out.steer(fr_hit)     = track_in.meas{fr_hit}.steer(1);
        track_out.time(fr_hit)      = track_in.meas{fr_hit}.time;
        fr_hit = fr_hit+1;
    end
    
end

%% Save to file

% Create folder
foldername = ['Results/Tracking/', scenario.simsetup.filename];
if ~exist(foldername, 'dir')
    mkdir(foldername);
end

% Save file
filename = [foldername, sprintf('/Unit_%d.mat', unit)];
save(filename, 'track_out');

% Read out to command window
disp(['Tracking results saved in ', filename]);

end

