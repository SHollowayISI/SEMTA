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
detection.detect_cube = zeros(sz(1), sz(2), num_loops);

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
            detection.detect_cube(:,:,loop) = (rd_cube > abs_thresh);
            
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
            detection.detect_cube(rng_ax,dop_ax,loop) = reshape(cfar_out, length(rng_ax), length(dop_ax));          
            
    end
    
    %% Binary Integration
    switch radarsetup.int_type
        case 'binary'
            % Total up number of detection hits
            count_cube = sum(detection.detect_cube, 3);

            % Perform binary m-of-n integration
            detection.int_detect_cube = logical(count_cube >= radarsetup.det_m);

            % Generate cube of average power for multiple-detection indices
            avg_cube = squeeze((detection.int_detect_cube .* sum(detection.detect_cube .* cube.pow_cube, 3)) ./ count_cube);
            
        case 'incoherent'
            % Use CFAR result for integrated cube
            detection.int_detect_cube = detection.detect_cube;
            
            % Generate power cube
            avg_cube = squeeze(sum(detection.int_detect_cube .* cube.pow_cube, 3));
    end
end

%% Calculate properties of detection

sum_cube = sum(avg_cube, 3);
mono_sum_cube = sum(sqrt(avg_cube), 3);
mono_diff_cube = diff(sqrt(avg_cube), 1, 3);
ratio_cube = mono_diff_cube ./ mono_sum_cube;

% Determine if any target is detected
detection.detect_logical = any(detection.int_detect_cube > 0, 'all');

%% Estimate Target Coordinates

% Find connected objects in R-D cube
cc = bwconncomp(detection.int_detect_cube);
regions = regionprops(cc, sum_cube, 'WeightedCentroid');

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
    
    % Collect ratio and power sum in this region
    power_list = sum_cube(cc.PixelIdxList{n});
    ratio_list = ratio_cube(cc.PixelIdxList{n});
    
    % Estimate angle-of-arrival using amplitude comparison monopulse
    rat = sum(ratio_list .* power_list) ./ sum(power_list);
    monopulse_aoa = (cosd(scenario.multi.steering_angle(scenario.flags.frame, scenario.flags.unit))^-2) * ...
        radarsetup.beamwidth * rat / radarsetup.mono_coeff;
    detection.detect_list.az(end+1) = monopulse_aoa ...
        + scenario.multi.steering_angle(scenario.flags.frame, scenario.flags.unit);
    
    % Calculate range estimate and correction
    if radarsetup.range_off
        rangeMeas = interp1(cube.range_axis, regions(n).WeightedCentroid(2));
        nearest = scenario.cube.range_res * floor(rangeMeas / scenario.cube.range_res);
        resid = rangeMeas - nearest;
        offset = interp1(offsetAxis, offsetCurve, resid, 'linear', 'extrap');
        rangeCalc = nearest + offset;
    else
        rangeCalc = interp1(cube.range_axis, regions(n).WeightedCentroid(2));
    end
        
    % Store direct coordinates
    detection.detect_list.range(end+1) = rangeCalc;
    detection.detect_list.vel(end+1) = interp1(cube.vel_axis, regions(n).WeightedCentroid(1));
    
    % Store derived coordinates
    detection.detect_list.cart(:,end+1) = detection.detect_list.range(end) * ...
        [cosd(detection.detect_list.az(end)); sind(detection.detect_list.az(end))];
    
    % Store SNR
    detection.detect_list.SNR(end+1) = 10*log10(max(power_list, [], 'all')) ...
        - detection.noise_pow;
end

end