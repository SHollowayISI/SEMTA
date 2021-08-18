function [track] = KalmanFilter_SingleUnit(track, rs, ts, frame)
%KALMANFILTER_SINGLEUNIT Kalman tracking filter for single unit tracking.
%   Takes as input track data structure, radarsetup,
%   radarsetup.tracking_single, and frame number.

%% Unpack Variables

meas = track.meas{frame};

%% Evaluate Track History

% Determine previous successful measurement
hit_ind = find(track.hit_list(1:(frame-1)));
hit_ind = hit_ind((frame - hit_ind) <= (ts.miss_max+1));

% Check if track needs to be initialized
if isempty(hit_ind)
    
    % Use measurement as state prediction
    X_init = [...
        meas.cart(1);  ...
        meas.vel * cosd(meas.az); ...
        meas.cart(2); ...
        meas.vel * sind(meas.az)];
         
    % Use measurement uncertainties as covariance prediction
    if ts.EKF
        speed_unc = sqrt((ts.max_vel)^2 / 3);
        az_rad = deg2rad(meas.az);
        P_init = sqrt(diag([ ...
            (ts.sigma_z_EKF(1)*cos(az_rad))^2 ...
                + (ts.sigma_z_EKF(2)*meas.range*sin(az_rad))^2, ...
            (ts.sigma_z_EKF(3)*cos(az_rad))^2 ...
                + (speed_unc*meas.range*sin(az_rad))^2 ...
                + (ts.sigma_z_EKF(2)*meas.vel*sin(az_rad))^2, ...
            (ts.sigma_z_EKF(1)*sin(az_rad))^2 ...
                + (ts.sigma_z_EKF(2)*meas.range*cos(az_rad))^2, ...
            (ts.sigma_z_EKF(3)*sin(az_rad))^2 ...
                + (speed_unc*meas.range*cos(az_rad))^2 ...
                + (ts.sigma_z_EKF(2)*meas.vel*cos(az_rad))^2]));
    else
        P_init = diag([sigma_z(1), 0, sigma_z(2), 0]);
    end
        
    % Save results
    track = saveStepData(track, frame, X_init, P_init, X_init, P_init);
    
    % Exit step
    return;
    
else
    
    % Calculate timestep since previous hit
    last_hit_frame = hit_ind(end);
    Tm = rs.frame_time * (frame - last_hit_frame);
    
end

%% Calculate Model Matrices

% Process covariance matrix
if ts.EKF
    R = (ts.sigma_z_EKF.^2) .* eye(3);
else
    R = (ts.sigma_z.^2) .* eye(2);
end

% Process covariance matrix (DWNA assumption)
Q_1d = [(Tm^4)/4, (Tm^3)/2; ...
        (Tm^3)/2, (Tm^2)];
Q = [Q_1d*(ts.sigma_v(1)^2), zeros(2); ...
    zeros(2), Q_1d*(ts.sigma_v(2)^2)];

% Kinematic process matrix
F_1d = [1, Tm; 0, 1];
F = [F_1d, zeros(2); ...
    zeros(2), F_1d];

%% Prediction Step

% Predicted kinematic vector
X_pre = F * track.estimate{last_hit_frame}.state;

% Predicted kinematic covariance
P_pre = F * (track.estimate{last_hit_frame}.covar * F') + Q;

% Save prediction as estimate if measurement was not taken
if ~track.hit_list(frame)
    track = saveStepData(track, frame, X_pre, P_pre, X_pre, P_pre);
    return;
end

%% Calculate State Variables

% Measurement vector
if ts.EKF
    Z = [meas.range; deg2rad(meas.az); meas.vel];
else
    Z = meas.cart;
end

%% Calculate Measurement Matrices

% Measurement matrix and measurement residual
if ts.EKF

    % Measurement residual
    h_r = sqrt(X_pre(1)^2 + X_pre(3)^2);
    h_b = atan(X_pre(3)/X_pre(1));
    h_d = (X_pre(1)*X_pre(2) + X_pre(3)*X_pre(4)) / h_r;
    
    % Range-Doppler coupling
    RDtau = (rs.f_c - 0.5 * rs.bw) * rs.t_p / rs.bw;
    h_rd = h_r + RDtau * h_d;
    h = [h_rd; h_b; h_d];
    
    Z_res = Z - h;
    
    % Measurement matrix
    H_r = [X_pre(1), 0, X_pre(3), 0]/h_r;
    H_b = [-X_pre(3), 0, X_pre(1), 0]/(h_r^2);
    H_d(1) = ((X_pre(3)^2)*X_pre(2) - (X_pre(3)*X_pre(4))*X_pre(1)) / ...
        (h_r^3);
    H_d(2) = X_pre(1)/h_r;
    H_d(3) = ((X_pre(1)^2)*X_pre(4) - (X_pre(1)*X_pre(2))*X_pre(3)) / ...
        (h_r^3);
    H_d(4) = X_pre(3)/h_r;
    
    % Range-Doppler coupling
    H_rd = H_r + RDtau * H_d;
    
    H = [H_rd; H_b; H_d];
    
else
    
    % Measurement matrix
    H = [1, 0, 0, 0; ...
         0, 0, 1, 0];
    
    % Measurement residual
    Z_res = Z - (H * X_pre);
    
end

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
track = saveStepData(track, frame, X_est, P_est, X_pre, P_pre);

%% Function Declarations

% Function to save estimated and predicted data
function [track] = saveStepData(track, frame, X_est, P_est, X_pre, P_pre)
    
    % Save estimates
    track.estimate{frame}.state = X_est;
    track.estimate{frame}.covar = P_est;

    % Save human readable data
    track.estimate{frame}.range = sqrt(X_est(1)^2 + X_est(3)^2);
    track.estimate{frame}.speed = sqrt(X_est(2)^2 + X_est(4)^2);
    track.estimate{frame}.cart = X_est([1; 3]);
    track.estimate{frame}.az = atand(X_est(3) / X_est(1));

    % Save predictions
    track.prediction{frame}.state = X_pre;
    track.prediction{frame}.covar = P_pre;

    % Save human readable data (prediction)
    track.prediction{frame}.range = sqrt(X_pre(1)^2 + X_pre(3)^2);
    track.prediction{frame}.speed = sqrt(X_pre(2)^2 + X_pre(4)^2);
    track.prediction{frame}.cart = X_pre([1; 3]);
    track.prediction{frame}.az = atand(X_pre(3) / X_pre(1));
    
end

end

