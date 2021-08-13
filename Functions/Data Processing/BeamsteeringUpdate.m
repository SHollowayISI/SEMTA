function [angle_out] = BeamsteeringUpdate(scenario, readout)
%BEAMSTEERINGUPDATE Calculate new beam steering angle
%   Calculate new beam steering angle, depending on detections and 
%   current radar mode.

%% Default Variable Values

if ~exist('readout', 'var')
    readout = true;
end

%% Unpack Variables

rs = scenario.radarsetup;
unit = scenario.flags.unit;
frame = scenario.flags.frame;
mode = scenario.flags.mode;

%% Angle Calculation

% Split depending on radar mode
switch mode
    case 'wait'
        % Use idle angle
        angle_out = rs.modes.wait.init_angle;
        
    case 'search'
        if frame == 1
            % Use initial angle
            angle_out = rs.modes.search.init_angle;
        else
            % Unpack variables
            last_angle = scenario.multi.steering_angle(frame-1, unit);
            step_angle = rs.modes.search.search_step;
            max_angle = rs.modes.search.search_max;
            
            % Round to value, in case of mode change
            last_angle = step_angle * round(last_angle / step_angle);
            
            % Update angle by search step
            new_angle = last_angle + step_angle;
            
            % Wrap around if angle is too large
            if abs(new_angle) > max_angle
                angle_out = -1 * sign(step_angle) * max_angle;
            else
                angle_out = new_angle;
            end
        end
        
    case 'track'
        if frame == 1
            % Use initial angle
            angle_out = rs.modes.wait.init_angle;
        else
            
            % Unpack variables
            det_list = scenario.detection.detect_list;
            last_angle = scenario.multi.steering_angle(frame-1, unit);
            
            % Check for detections in previous frame
            if scenario.detection.detect_logical
                
                % Search for highest SNR detection
                [~, max_ind] = max(det_list.SNR);
                
                % Update using previous monopulse angle
                angle_out = det_list.az(max_ind);
            else
                
                % Use previous angle
                %TODO: USE TRACKING ESTIMATE IF AVAILABLE
                angle_out = last_angle; 
            end 
        end
        
    case 'ideal' 
        % Calculate mid-frame time
        t_st = rs.frame_time * (frame - 0.5);

        % Set radar/target position and velocity
        tx_pos = scenario.multi.radar_pos(:,unit);
        tgt_pos = scenario.traj.pos(scenario.traj, t_st);

        % Calculate perfect angle to target
        [~, tgt_ang] = rangeangle(tgt_pos, tx_pos);
        angle_out = tgt_ang(1);
end

%% Read out update

if readout
        fprintf('Steering angle set to %0.1f deg\n', angle_out);
end

end

