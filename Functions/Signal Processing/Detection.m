function [detection] = Detection(scenario)
%DETECTION_SEMTA Performs target detection for SEMTA project
%   Takes scenario object as input, provides scenario.detection object as
%   output, containing information about detected targets.

%% Unpack Variables

radarsetup = scenario.radarsetup;
cube = scenario.cube;

%% Perform Detection

% Estimate noise power
detection.noise_pow = pow2db(median(mean(sum(cube.pow_cube, 4), 1), 'all'));

% Generate detection map
sz = size(cube.pow_cube);
sz = sz(1:2);

% Set up CFAR indices
if strcmp(radarsetup.detect_type, 'CFAR')
    
            % Set up range and Doppler axes
            pad = radarsetup.num_guard + radarsetup.num_train;
            rng_ax = (pad(1) + 1):(sz(1)-pad(1));
            dop_ax = (pad(2) + 1):(sz(2)-pad(2));
            
            % Gneerate indices from axes
            idx = [];
            idx(1,:) = repmat(rng_ax, 1, length(dop_ax));
            idx(2,:) = reshape(repmat(dop_ax, length(rng_ax), 1), 1, []);
            
end


% Branch depending on integration type
switch radarsetup.int_type
    case 'binary'
        num_loops = radarsetup.cpi_fr;
    case 'incoherent'
        num_loops = 1;
end

detection.single_ind_list = cell(num_loops, 1);
detection.pow_list = cell(num_loops, 1);
detect_cube = zeros(sz(1), sz(2), num_loops);

% Loop across CPIs
for loop = 1:num_loops
    
    % Branch depending on integration type
    switch radarsetup.int_type
        case 'binary'
            
            % Set cube to examine
            rd_cube = sum(cube.pow_cube(:,:,loop,:), 4);
            
            % Estimate noise power
            detection.noise_pow = pow2db(median(mean(sum(cube.pow_cube, 4), 1), 'all'));

        case 'incoherent'
            
            % Set cube to examine
            rd_cube = sum(cube.pow_cube, [3 4]);
            
            % Estimate noise power
            detection.noise_pow = pow2db(median(mean(rd_cube, 1), 'all'));

    end
    
    switch radarsetup.detect_type
        case 'threshold'
            %% Perform Threshold Detection
            
            % Calculate threshold in absolute
            abs_thresh = db2pow(radarsetup.thresh + detection.noise_pow);
            
            % Perform detection
            detect_cube(:,:,loop) = (rd_cube > abs_thresh);
            
        case 'CFAR'
            %% Perform CFAR Detection
            
            % Use parallel processing if available and desired
            if scenario.simsetup.par_cfar
                
                % Perform CFAR detection
                CFAR = scenario.sim.CFAR;
                stride = 17603;
                steps = ceil(size(idx,2) / stride);
                detection_list = nan(stride, steps);
                
                parfor i = 1:steps
                    inds = (stride*(i-1) + 1):(stride*i)
                    CFAR_detect = CFAR(rd_cube, idx(:,inds));
                    detection_list(:,i) = CFAR_detect;
                end
                cfar_out = detection_list(:);
                
            else
                
                % Use single threaded processing otherwise
                cfar_out = scenario.sim.CFAR(rd_cube, idx);
            end
            
            % Save detection cube
            detect_cube(rng_ax,dop_ax,loop) = reshape(cfar_out, length(rng_ax), length(dop_ax)); 
    end
    
    % Save detection lists
    detection.single_ind_list{loop} = find(detect_cube(:,:,loop));
    [rngI, dopI] = ind2sub(sz, detection.single_ind_list{loop});
    for n = 1:2
        linI = sub2ind(size(cube.pow_cube), rngI, dopI, loop*ones(size(rngI)), n*ones(size(rngI)));
        detection.pow_list{loop}(:,n) = cube.pow_cube(linI);
    end
    
end
        
%% Binary Integration
switch radarsetup.int_type
    case 'binary'
        
        % Concatenate all result lists
        concat_list = detection.single_ind_list{:};
        concat_pow_list = detection.pow_list{:};
        unique_list = unique(concat_list);
        
        % Loop through all unique detections
        detection.combined_ind_list = [];
        avg_list = nan(length(unique_list), 2);
        for n = 1:length(unique_list)
            
            % Count instances
            ind = unique_list(n);
            count = sum(concat_list == ind);
            
            % Continue if not enough detections
            if count < radarsetup.det_m
                continue;
            end
            
            % Update lists
            detection.combined_ind_list(end+1) = ind;
            avg_list(n,:) = sum(concat_pow_list(concat_list == ind)) / count;
            
        end
        
        % Remove NaNs
        avg_list = avg_list(~isnan(avg_list(:,1)),:);
        
    case 'incoherent'
        
        % Use CFAR result for integrated cube
        detection.combined_ind_list = detection.single_ind_list{1};
        
        % Generate power cube
        avg_list = detection.pow_list{1};
end

% Determine if any target is detected
detection.detect_logical = ~isempty(detection.combined_ind_list);


%% Find connected components

% Initialize labels
label_list = nan(size(detection.combined_ind_list));
equiv_list = {};

% Loop through list of detections (First pass)
for n = 1:length(detection.combined_ind_list)
    
    % Calculate subscript indices
    lin_ind = detection.combined_ind_list(n);
    [r_ind, d_ind] = ind2sub(sz, lin_ind);
    
    % Check index to the left
    if (r_ind > 1) && (r_ind <= sz(1))
        left_ind = sub2ind(sz, r_ind-1, d_ind);
        left_list_ind = find(detection.combined_ind_list == left_ind);
        left_label = label_list(left_list_ind);
    else
        left_list_ind = [];
    end
    
    % Check index above
    if (d_ind > 1) && (d_ind <= sz(2))
        up_ind = sub2ind(sz, r_ind, d_ind-1);
        up_list_ind = find(detection.combined_ind_list == up_ind);
        up_label = label_list(up_list_ind);
    else
        up_list_ind = [];
    end
    
    % Condition 1: Left is true
    if ~isempty(left_list_ind)
        
        % Use left label for CUT
        label_list(n) = left_label;
    end
    
    % Condition 2: Left and Up are true but different labels
    if ~isempty(left_list_ind) && ~isempty(up_list_ind) && (left_label ~= up_label)
        
        % Merge Left and Up
        label_list(n) = min(left_label, up_label);
        
        % Add equivalence
        equiv_list{left_label}(end+1) = up_label;
        equiv_list{up_label}(end+1) = left_label;
        
    end
    
    % Condition 3: Up is true and Left is not
    if ~isempty(up_list_ind) && isempty(left_list_ind)
        
        % Use up label for CUT
        label_list(n) = up_label;
    end
    
    % Condition 4: Neither are true
    if isempty(left_list_ind) && isempty(up_list_ind)
        
        % Add new label
        new_label = length(equiv_list) + 1;
        equiv_list{new_label} = new_label;
        label_list(n) = new_label; 
    end
    
end

% Consolidate equivalences
for n = 1:length(equiv_list)
    for m = (n+1):length(equiv_list)
        if ~isempty(intersect(equiv_list{n}, equiv_list{m}))
            equiv_list{n} = unique(union(equiv_list{n}, equiv_list{m}));
            equiv_list{m} = equiv_list{n};
        end
    end
end


% Loop through list of detections (Second pass)
for n = 1:length(detection.combined_ind_list)
    
    % Assign lowest label to true detections
    label_list(n) = min(equiv_list{label_list(n)});
    
end

%% Separate data by regions

% Generate region struct
unique_list = unique(label_list);
regions = struct( ...
    'pixelIdxList',     cell(length(unique_list),1), ...
    'rngIdxList',       cell(length(unique_list),1), ...
    'dopIdxList',       cell(length(unique_list),1), ...
    'powerList',        cell(length(unique_list),1), ...
    'ratioList',        cell(length(unique_list),1), ...
    'weightedCentroid', cell(length(unique_list),1));

% Calculate region information
for n = 1:length(regions)
    
    % Lists of region indices
    regions(n).pixelIdxList = detection.combined_ind_list(label_list == unique_list(n));
    [regions(n).rngIdxList, regions(n).dopIdxList] = ind2sub(sz, regions(n).pixelIdxList);
    
    % Derived lists of power values
    regionAvgList = avg_list(label_list == unique_list(n),:);
    regions(n).powerList = sum(regionAvgList,2);
    regions(n).ratioList = diff(sqrt(regionAvgList), 1, 2) ./ sum(sqrt(regionAvgList), 2);
    
    % Centroid values
    powerSum = sum(regions(n).powerList);
    regions(n).weightedCentroid(1) = sum(regions(n).rngIdxList .* regions(n).powerList) / powerSum;
    regions(n).weightedCentroid(2) = sum(regions(n).dopIdxList .* regions(n).powerList) / powerSum;
    regions(n).weightedCentroid(3) = sum(regions(n).ratioList .* regions(n).powerList) / powerSum;
    
end

%% Estimate Target Coordinates

% Generate list of detection coordinates
detection.detect_list.range = [];
detection.detect_list.vel = [];
detection.detect_list.az = [];
detection.detect_list.cart = [];
detection.detect_list.SNR = [];
detection.detect_list.num_detect = length(regions);

% Load offset curve
if radarsetup.range_off
    loadIn = load('Results\Error Curves\RangeErrorCurveFixedWindow.mat', 'offsetAxis', 'offsetCurve');
    offsetCurve = loadIn.offsetCurve;
    offsetAxis = loadIn.offsetAxis;
end

% Determine Centroid of azimuth-elevation slice
for n = 1:length(regions)
    
    % Test coarse range calculation
    rangeMeas = interp1(cube.range_axis, regions(n).weightedCentroid(1));
    if (rangeMeas < radarsetup.rng_limits(1)) || (rangeMeas > radarsetup.rng_limits(2))
        detection.detect_list.num_detect = detection.detect_list.num_detect - 1;
        continue;
    end
    
    % Estimate angle-of-arrival using amplitude comparison monopulse
    monopulse_aoa = (cosd(scenario.multi.steering_angle(scenario.flags.frame, scenario.flags.unit))^-2) * ...
        regions(n).weightedCentroid(3) * radarsetup.beamwidth / radarsetup.mono_coeff;
    detection.detect_list.az(end+1) = monopulse_aoa ...
        + scenario.multi.steering_angle(scenario.flags.frame, scenario.flags.unit);
    
    % Calculate range estimate and correction
    if radarsetup.range_off
        nearest = scenario.cube.range_res * floor(rangeMeas / scenario.cube.range_res);
        resid = rangeMeas - nearest;
        offset = interp1(offsetAxis, offsetCurve, resid, 'linear', 'extrap');
        rangeCalc = nearest + offset;
    else
        rangeCalc = interp1(cube.range_axis, regions(n).weightedCentroid(1));
    end
        
    % Store direct coordinates
    detection.detect_list.range(end+1) = rangeCalc;
    detection.detect_list.vel(end+1) = -interp1(cube.vel_axis, regions(n).weightedCentroid(2));
    
    % Store derived coordinates
    detection.detect_list.cart(:,end+1) = detection.detect_list.range(end) * ...
        [cosd(detection.detect_list.az(end)); sind(detection.detect_list.az(end))];
    
    % Store SNR
    detection.detect_list.SNR(end+1) = 10*log10(max(regions(n).powerList, [], 'all')) ...
        - detection.noise_pow;
end

end