function [multi] = TargetTracking_SEMTA(scenario)
%TARGETTRACKING_SEMTA Performs radar target tracking for SEMTA project
%   Takes radar scenario object as input, returns modified scenario.multi
%   as output.

%% Unpack Variables

multi = scenario.multi;
simsetup = scenario.simsetup;

%% Apply Tracking

switch multi.track_method
    
    case 'Moving Average'
        % Moving Average Filter Smoothing
        for dim = 1:2
            multi.track_points(dim,:) = movmean(multi.lat_points(dim,:), simsetup.wind_size(dim));
        end
        
    case 'Kalman Filter'
        % Kalman Filter Estimation
        
        % Determine location of detections
        index = 1:multi.n_fr;
        det_index = index(~isnan(multi.lat_points(1,:)));
        miss_index = index(isnan(multi.lat_points(1,:)));
        diff_index = diff(det_index);
        
        % Interpolate between missing points if desired
        multi.lat_points_calc = scenario.multi.lat_points;
        if multi.interp
            for dim = 1:2
                multi.lat_points_calc(dim,miss_index) = ...
                    interp1(det_index, multi.lat_points(dim,det_index), miss_index, 'spline');
            
                det_index = index;
                diff_index = diff(det_index);
            end
        end
        
        % Calculate parameters
        T = simsetup.n_p*simsetup.cpi_fr/simsetup.prf;
        
        % Set up variance values
        sig_v = multi.sigma_v;
        sig_z = multi.sigma_z;
        
        % Calculate process covariance matrix
        R = [sig_z, 0; 0, sig_z];
        
        % Calculate measurement matrix
        H = [1, 0, 0, 0; 0, 0, 1, 0];
        
        % Pre-allocate memory for estimation and prediction variables
        x_est = nan(4, multi.n_fr);
        x_pre = nan(4, multi.n_fr);
        
        % Initialization of estimation variables
        k = det_index(1);
        x_est(:,k) = ...
            [multi.lat_points_calc(1,k); multi.lat_points_calc(1,k)-multi.lat_points_calc(1,k); ...
            multi.lat_points_calc(2,k); multi.lat_points_calc(2,k)-multi.lat_points_calc(2,k)];
        
        % Initalize covariance matrix
        P_est = eye(4);
        
        % Kalman filter loop
        for k = 2:length(det_index)
            
            % Determine time difference between points
            T_m = T * diff_index(k-1);
            
            % Calculate DWNA Covariance matrix
            Q_1d = [(T_m^4)/4, (T_m^3)/2; ...
                (T_m^3)/2, (T_m^2)]*sig_v^2;
            Q = [Q_1d, zeros(2); zeros(2), Q_1d];
            
            % Calculate kinematic process matrix
            F_1d = [1, T_m; 0, 1];
            F = [F_1d, zeros(2); zeros(2), F_1d];
            
            % Predict current target location from previous
            x_pre(:,det_index(k)) = F * x_est(:,det_index(k-1));
            
            % Predict covariance matrix
            P_pre = F * P_est * F' + Q;
            
            % Calculate measurement residual covariance
            S = H * P_pre * H' + R;
            
            % Calculate Kalman matrix
            K = P_pre * H' / S;
            
            % Estimate covariance matrix
            P_est = P_pre - K * H * P_pre;
            
            % Estimate current target location from Kalman filter
            x_est(:,det_index(k)) = ...
                x_pre(:,det_index(k)) + K * (...
                multi.lat_points_calc(:,det_index(k)) - H * x_pre(:,det_index(k)));
            
        end
        
        % Save tracked points
        multi.track_points = [x_est(1,:); x_est(3,:)];
        
    case 'None'
        % No Track Filter Applied
        multi.track_points = multi.lat_points;
end

end

