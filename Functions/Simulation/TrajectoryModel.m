function [traj_out] = TrajectoryModel(traj_in)
%TRAJECTORYMODEL Models target trajectory for TRASAT-SEMTA
%   Takes as input scenario.traj struct, outputs scenario.traj struct with
%   modified 'position', 'velocity', and 'trajectory' fields

%% Model Trajectory

% Transfer values
traj_out = traj_in;

switch traj_out.model
    case 'static'
        % Set position and velocity to static input
        traj_out.pos = repmat(traj_out.pos_st,1,length(traj_out.time));
        traj_out.vel = repmat(traj_out.vel_st,1,length(traj_out.time));
        
    case 'model'
        % Calculate position
        traj_out.pos = ...
            [traj_out.exc * sin(traj_out.per * traj_out.time); ...
            traj_out.yvel * traj_out.time; ...
            ones(size(traj_out.time)) * traj_out.alt];
        
        % Calculate velocity
        traj_out.vel = ...
            [traj_out.per * traj_out.exc * cos(traj_out.per * traj_out.time); ...
            ones(size(traj_out.time)) * traj_out.yvel; ...
            zeros(size(traj_out.time))];
        
    case 'model_constant'
        % Calculate y-velocity
        yvel_mod = sqrt(traj_out.yvel.^2 - ...
            (traj_out.per * traj_out.exc * cos(traj_out.per * traj_out.time)).^2);
        
        % Calculate position
        traj_out.pos = ...
            [traj_out.exc * sin(traj_out.per * traj_out.time); ...
            cumsum(yvel_mod * (traj_out.time(2)-traj_out.time(1)) ...
            .* ones(size(traj_out.time))); ...
            ones(size(traj_out.time)) * traj_out.alt];
        
        % Calculate velocity
        traj_out.vel = ...
            [traj_out.per * traj_out.exc * cos(traj_out.per * traj_out.time); ...
            ones(size(traj_out.time)) .* yvel_mod; ...
            zeros(size(traj_out.time))];
        
    case 'linear'
        % Calculate position
        traj_out.pos = ...
            traj_out.pos_st + traj_out.time.*traj_out.vel_st;
        
        % Calculate velocity
        traj_out.vel = traj_out.vel_st.*ones(size(traj_out.time));
end

% Save trajectory in 7-column format used by Phased Array Toolbox
traj_out.full_traj = transpose([traj_out.time; traj_out.pos; traj_out.vel]);


end

