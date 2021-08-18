function [dist] = MahanalobisDistance(track, det_list, rs, ts, frame)
%MAHANALOBISDISTANCE Mahanalobis Distance calculation
%   Calculates statistical distance, weighted by uncertainty for tracking

%% Calculate Model Matrices

% Process covariance matrix
if ts.EKF
    R = (ts.sigma_z_EKF.^2) .* eye(3);
else
    R = (ts.sigma_z.^2) .* eye(2);
end

% Calculate time step
Tm = rs.frame_time;

% Process covariance matrix (DWNA assumption)
Q_1d = [(Tm^4)/4, (Tm^3)/2; ...
        (Tm^3)/2, (Tm^2)];
Q = [Q_1d*(ts.sigma_v(1)^2), zeros(2); ...
    zeros(2), Q_1d*(ts.sigma_v(2)^2)];

% Kinematic process matrix
F_1d = [1, Tm; 0, 1];
F = [F_1d, zeros(2); ...
    zeros(2), F_1d];

%% Calculate Predicted State

% Predicted kinematic vector
X_pre = F * track.estimate{frame-1}.state;

% Predicted kinematic covariance
P_pre = F * (track.estimate{frame-1}.covar * F') + Q;

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
end

%% Calculate State Variables


%% Calculate Mahanalobis Distance

% Calculate measurement covariance residual
S = H * P_pre * H' + R;

% Complete calculation for each measurement
dist = inf(det_list.num_detect,1);
for meas_ind = 1:det_list.num_detect
    
    % Calculate measurement vector and residual
    if ts.EKF
        Z = [det_list.range(meas_ind); deg2rad(det_list.az(meas_ind)); det_list.vel(meas_ind)];
        Z_res = Z - h;
    else
        Z = det_list.cart;
        Z_res = Z - (H * X_pre);
    end
    
    dist(meas_ind) = Z_res' * (S \ Z_res);
    
end


end

