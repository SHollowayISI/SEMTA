function [detection] = Detection(scenario)
%DETECTION_SEMTA Performs target detection for SEMTA project
%   Takes scenario object as input, provides scenario.detection object as
%   output, containing information about detected targets.

%% Unpack Variables

radarsetup = scenario.radarsetup;
cube = scenario.cube;

%% Perform Detection

% Estimate noise power
detection.noise_pow = pow2db(median(mean(cube.pow_cube, 1), 'all'));

% Generate detection map
sz = size(cube.pow_cube);
detection.detect_cube = zeros(sz(1:3));

% Set up CFAR indices
if strcmp(radarsetup.detect_type, 'CFAR')
    
            % Set up index map
            pad = radarsetup.num_guard + radarsetup.num_train;
            rng_ax = (pad(1) + 1):(sz(1)-pad(1));
            dop_ax = (pad(2) + 1):(sz(2)-pad(2));
            
            idx = [];
            idx(1,:) = repmat(rng_ax, 1, length(dop_ax));
            idx(2,:) = reshape(repmat(dop_ax, length(rng_ax), 1), 1, []);
            
            % Position in radar cube to place results
            cfar_indices_r = (pad(1)+1):(pad(1)+length(rng_ax));
            cfar_indices_d = (pad(2)+1):(pad(2)+length(dop_ax));
end

% Loop across CPIs
for cpi = 1:radarsetup.cpi_fr
    
    rd_cube = sum(cube.pow_cube(:,:,cpi,:), 4);
    
    switch radarsetup.detect_type
        case 'threshold'
            %% Perform Threshold Detection
            
            % Calculate threshold in absolute
            abs_thresh = db2pow(radarsetup.thresh + detection.noise_pow);
            
            % Perform detection
            detection.detect_cube(:,:,cpi) = (rd_cube > abs_thresh);
            
        case 'CFAR'
            %% Perform CFAR Detection
            
            % Perform CFAR detection
            cfar_out = scenario.sim.CFAR(rd_cube, idx);
            
            % Save detection cube
            detection.detect_cube(cfar_indices_r,cfar_indices_d,cpi) = reshape(cfar_out, length(rng_ax), length(dop_ax));          
            
    end
end

%% Calculate properties of detection

% Total up number of detection hits
count_cube = sum(detection.detect_cube, 3);

% Perform binary m-of-n integration
detection.cfar_detect_cube = logical(count_cube >= radarsetup.det_m);

% Generate cube of average power for multiple-detection indices
avg_cube = squeeze((detection.cfar_detect_cube .* sum(detection.detect_cube .* cube.pow_cube, 3)) ./ count_cube);
sum_cube = sum(avg_cube, 3);
mono_sum_cube = sum(sqrt(avg_cube), 3);
mono_diff_cube = diff(sqrt(avg_cube), 1, 3);
ratio_cube = mono_diff_cube ./ mono_sum_cube;

% Determine if any target is detected
detection.detect_logical = any(detection.cfar_detect_cube > 0, 'all');

%% Estimate Target Coordinates

% Find connected objects in R-D cube
cc = bwconncomp(detection.cfar_detect_cube);
regions = regionprops(cc, sum_cube, 'WeightedCentroid');

% Generate list of detection coordinates
detection.detect_list.range = [];
detection.detect_list.vel = [];
detection.detect_list.az = [];
detection.detect_list.cart = [];
detection.detect_list.SNR = [];
detection.detect_list.num_detect = length(regions);

% Determine Centroid of azimuth-elevation slice
for n = 1:length(regions)
    
    % Collect ratio and power sum in this region
    power_list = sum_cube(cc.PixelIdxList{n});
    ratio_list = ratio_cube(cc.PixelIdxList{n});
    
    % Estimate angle-of-arrival using amplitude comparison monopulse
    rat = sum(ratio_list .* power_list) ./ sum(power_list);
    detection.detect_list.az(end+1) = radarsetup.beamwidth * rat / radarsetup.mono_coeff;
    
    % Store direct coordinates
    detection.detect_list.range(end+1) = interp1(cube.range_axis, regions(n).WeightedCentroid(2));
    detection.detect_list.vel(end+1) = interp1(cube.vel_axis, regions(n).WeightedCentroid(1));
    
    % Store derived coordinates
    detection.detect_list.cart(:,end+1) = detection.detect_list.range(end) * ...
        [cosd(detection.detect_list.az(end)); sind(detection.detect_list.az(end))];
    
    % Store SNR
    detection.detect_list.SNR(end+1) = 10*log10(max(power_list, [], 'all')) ...
        - detection.noise_pow;
end

end
