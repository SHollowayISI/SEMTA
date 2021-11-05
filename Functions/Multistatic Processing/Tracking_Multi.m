function [tracking_multi] = Tracking_Multi(scenario, passDirection)
%TRACKING_MULTI Perform tracking between frames of combined sensor data
%   Takes scenario object as input and returns smoothed list of track
%   estimates.

%% Unpack variables

tracking_multi = scenario.tracking_multi;
multi = scenario.multi;
rs = scenario.radarsetup;
tsm = rs.tracking_single;

% Set up new structs
tracking_multi.track_estimate = cell(tracking_multi.num_fr, 1);
tracking_multi.track_prediction = cell(tracking_multi.num_fr, 1);
tracking_multi.hit_list_out = true(tracking_multi.num_fr, 1);


%% Target Tracking

% Determine pass direction
if strcmp(passDirection, 'forward')
    fr_ind = 1:tracking_multi.num_fr;
elseif strcmp(passDirection, 'reverse')
    fr_ind = tracking_multi.num_fr:-1:1;
end

% Loop through frames
for fr = fr_ind
    
    % Determine previous successful measurement
    if strcmp(passDirection, 'forward')
        hit_ind = 1:(fr-1);
    elseif strcmp(passDirection, 'reverse')
        hit_ind = tracking_multi.num_fr:-1:(fr+1);
    end
    
    % If no track has been established
    if isempty(hit_ind)
            
        % Use measurement as state prediction
        X_init = tracking_multi.state{fr};
        P_init = tracking_multi.var{fr};
        
        % Save results
        tracking_multi = ...
            saveStepData(tracking_multi, fr, X_init, P_init, X_init, P_init);
            
        
    else
        
        % Calculate timestep since previous hit
        last_fr = hit_ind(end);
        Tm = tracking_multi.time(fr) - tracking_multi.time(last_fr);
        
        %% Kalman Filter Setup
        
        % Calculate model matrices
        R = tracking_multi.var{fr};

        % % Process covariance matrix (DWNA assumption)
        % Q_1d = [(Tm^4)/4, (Tm^3)/2; ...
        %         (Tm^3)/2, (Tm^2)];
        % Q = [Q_1d*(ts.sigma_v(1)^2), zeros(2); ...
        %     zeros(2), Q_1d*(ts.sigma_v(2)^2)];
        %
        % % Kinematic process matrix
        % F_1d = [1, Tm; 0, 1];
        % F = [F_1d, zeros(2); ...
        %     zeros(2), F_1d];
        
        % Process covariance matrix (NCA model)
        Q_1d = [(Tm^4)/4, (Tm^3)/2, (Tm^2)/2; ...
            (Tm^3)/2, (Tm^2),   (Tm); ...
            (Tm^2)/2, (Tm),     1];
        Q = [Q_1d*(tsm.sigma_v_multi(1)^2), zeros(size(Q_1d)); ...
            zeros(size(Q_1d)), Q_1d*(tsm.sigma_v_multi(2)^2)];
        
        % Kinematic process matrix
        F_1d =  [1,     (Tm),   (Tm^2)/2; ...
            0,     1,      (Tm); ...
            0,     0,      1];
        F = [F_1d, zeros(size(F_1d)); ...
            zeros(size(F_1d)), F_1d];
        
        %% Prediction Step
        
        % Predicted kinematic vector
        X_pre = F * tracking_multi.track_estimate{last_fr}.state;
        
        % Predicted kinematic covariance
        P_pre = F * (tracking_multi.track_estimate{last_fr}.covar * F') + Q;
        
        %% Calculate Measurement Variables
        
        % Measurement vector
        Z = tracking_multi.state{fr};
        
        % Measurement matrix
        H = eye(6);
        
        % Measurement residual
        Z_res = Z - (H * X_pre);
        
        %% Estimation Step
        
        % Measurement residual covariance
        S = H * (P_pre * H') + R;
        
        % Kalman matrix
        K = P_pre * H' / S;
        
        % Estimated kinematic vector
        X_est = X_pre + K * Z_res;
        
        % Estimated kinematic covariance
        P_est = P_pre - K * (H * P_pre);
        
        % Save results
        tracking_multi = ...
            saveStepData(tracking_multi, fr, X_est, P_est, X_pre, P_pre);
        
    end
end

%% Function Declarations

% Function to save estimated and predicted data
function [tm] = saveStepData(tm, fr, X_est, P_est, X_pre, P_pre)
    
    % Set up structs
    tm.track_estimate{fr} = struct('state', [], 'covar', []', 'pos', [], 'vel', []);
    tm.track_prediction{fr} = struct('state', [], 'covar', []', 'pos', [], 'vel', []);
    
    % Save estimates
    tm.track_estimate{fr}.state = X_est;
    tm.track_estimate{fr}.covar = P_est;
    
    % Save human readable data
    tm.track_estimate{fr}.pos = X_est([1; 4]);
    tm.track_estimate{fr}.vel = X_est([2; 5]);
    
    % Save predictions
    tm.track_prediction{fr}.state = X_pre;
    tm.track_prediction{fr}.covar = P_pre;
    
    % Save human readable data (prediction)
    tm.track_prediction{fr}.pos = X_pre([1; 4]);
    tm.track_prediction{fr}.vel = X_pre([2; 5]);
    
end

end

