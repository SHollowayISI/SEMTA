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
    yvel_mod = sqrt(traj.yvel.^2 - ...
        (traj.per * traj.exc * cos(traj.per * time)).^2);
    currentPos = ...
        [traj.exc * sin(traj.per * time); ...
        cumsum(yvel_mod * (time(2)-time(1)) .* ones(size(time))); ...
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

