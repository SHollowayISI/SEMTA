function [multi] = MultistaticProcessing(scenario)
%MULTISTATICPROCESSING_SEMTA Multistatic processing for SEMTA project
%   Takes radar scenario object and returns list of target coordinates
%   estimated by multilateration.

%% Unpack Variables

multi = scenario.multi;

%% Perform 2-D Multilateration

% Pre-allocate data struction
multi.lat_points = nan(2, multi.n_fr);

% Break if only one transceiver reporting data
if multi.n_re == 1
    return
end

% Determine indices of transceivers, depending on method
switch scenario.multi.method
    case 'SNR'
        
        % Remove non-detection values
        sort_SNR = multi.SNR;
        sort_SNR(multi.detect == 0) = -Inf;
        
        % Sort SNR values
        [~, sort_ind] = sort(sort_SNR, 2, 'descend');
        multi.lat_index = sort(sort_ind(:,1:2), 2);
        
    case 'Range'
        
        % Remove non-detection values
        sort_range = multi.ranges;
        sort_range(multi.detect == 0) = Inf;
        
        % Sort Range values
        [~, sort_ind] = sort(sort_range, 2, 'ascend');
        multi.lat_index = sort(sort_ind(:,1:2), 2);
        
    otherwise
        % Default to first two transceivers
        multi.lat_index = ones(multi.n_fr, 2.*[1 2]);
end


% Loop through each frame of simulation
for frame = 1:multi.n_fr
    
    % Check if 2 radar units detected targets
    if nnz(scenario.multi.detect(frame,:)) < 2
        % Return NaN if multilateration can not be completed
        multi.lat_points(:,frame) = nan(2,1);
        
    else
        % Calculate multilateration results
        multi.lat_points(:,frame) = Multilateration2D( ...
            multi.radar_pos(1:2, multi.lat_index(frame, 1)), ...
            multi.ranges(frame, multi.lat_index(frame,1)), ...
            multi.radar_pos(1:2, multi.lat_index(frame, 2)), ...
            multi.ranges(frame, multi.lat_index(frame,2)));
    end
    
end

end

