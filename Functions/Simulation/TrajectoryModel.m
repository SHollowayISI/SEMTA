function [traj_out] = TrajectoryModel(traj_in)
%TRAJECTORYMODEL Models target trajectory for TRASAT-SEMTA
%   Takes as input scenario.traj struct, outputs scenario.traj struct with
%   modified 'position', 'velocity', and 'trajectory' fields

%% Static Trajectory Functions

% Position function
function currentPos = positionStatic(traj, time)
    currentPos = traj.pos_st .* ones(size(time));
end

% Velocity function
function currentVel = velocityStatic(traj, time)
    currentVel = traj.vel_st .* ones(size(time));
end

%% Oscillating Model Trajectory Functions

% Position function
function currentPos = positionModel(traj, time)
    currentPos = ...
        [traj.exc * sin(traj.per * time); ...
        traj.yvel * time; ...
        ones(size(time)) * traj.alt];
end

% Velocity function
function currentVel = velocityModel(traj, time)
   currentVel = ...
        [traj.per *traj.exc * cos(traj.per * time); ...
        ones(size(time)) *traj.yvel; ...
        zeros(size(time))];
end

%% Constant Velocity Oscillating Trajectory Functions

% Calculate position
function currentPos = positionModelConstant(traj, time)
    
    % Note: Complicated function due to the fact that integrating the
    % velocity function does not have a solution in terms of standard
    % functions
    a = traj.per;
    b = traj.exc;
    c = traj.yvel;
    x = a * time * 2 / pi;
%     phi = mod((x+1), 2) - 1;
    phi = mod(x + 1, 2) - 1;
    phi_int = x - phi;
    m = ((a*b)^2)/(a*a*b*b - c*c);
    e = phi_int * mEllipticE(m);
    yPos = e * (sqrt(c*c - a*a*b*b) / a);
    
    
    x_pos = a * sin(b * time);
    u = (c*c - a*a*b*b) / (a*a*b*b);
    pos = (a*b / 2) * (x_pos.*sqrt(u + x_pos.*x_pos) + u*log(sqrt(u+x_pos.*x_pos) + x_pos));
    yPos = yPos + pos;
    
%     yPos = (c^2 - (a*b)^2) * sqrt((a*a*b*b*cos(2*a*time) + a*a*b*b - 2*c*c) / (a*a*b*b - c*c)) ...
%         .* e ./ (a*sqrt(-a*a*b*b*cos(2*a*time) - a*a*b*b + 2*c*c));
    
    currentPos = ...
        [traj.exc * sin(traj.per * time); ...
        yPos; ...
        ones(size(time)) * traj.alt];
end

% Calculate velocity
function currentVel = velocityModelConstant(traj, time)
    yvel_mod = sqrt(traj.yvel.^2 - ...
        (traj.per * traj.exc * cos(traj.per * time)).^2);
    currentVel = ...
        [traj.per * traj.exc * cos(traj.per * time); ...
        ones(size(time)) .* yvel_mod; ...
        zeros(size(time))];
end

%% Linear Trajectory Functions

% Calculate position
function currentPos = positionLinear(traj, time)
    currentPos = traj.pos_st + time.*traj.vel_st;
end

% Calculate velocity
function currentVel = velocityLinear(traj, time)
    currentVel = traj.vel_st.*ones(size(time));
end

%% Model Trajectory

% Transfer values
traj_out = traj_in;

% Save position and velocity models
switch traj_out.model
    case 'static'
        traj_out.pos = @positionStatic;
        traj_out.vel = @velocityStatic;
    case 'model'
        traj_out.pos = @positionModel;
        traj_out.vel = @velocityModel;
    case 'model_constant'
        traj_out.pos = @positionModelConstant;
        traj_out.vel = @velocityModelConstant;
    case 'linear'
        traj_out.pos = @positionLinear;
        traj_out.vel = @velocityLinear;
end

% Save trajectory function
function fullTrajectory = fullTrajectory(traj, time)
    fullTrajectory = transpose([time; traj.pos(traj, time); traj.vel(traj, time)]); 
end

% Append trajectory function
traj_out.fullTrajectory = @fullTrajectory;

end

