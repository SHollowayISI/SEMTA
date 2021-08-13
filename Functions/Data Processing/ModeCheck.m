function [scenario] = ModeCheck(scenario, readout)
%MODECHECK Checks for radar mode change
%   Determines if system should change from one radar mode 
%   (such as search, track, idle) to another

%% Default Variable Values

if ~exist('readout', 'var')
    readout = true;
end

%% Unpack Variables

current_mode = scenario.flags.mode;
detected = scenario.detection.detect_logical;
mode_vars = scenario.radarsetup.modes;

%% Declare Flags

changed_mode = false;

%% Condition Checks

switch current_mode
    case 'wait'
        % Begin tracking if target is detected
        if detected
            changed_mode = true;
            new_mode = 'track';
        end
        
    case 'search'
        % Begin tracking if target is detected
        if detected
            changed_mode = true;
            new_mode = 'track';
        end
        
    case 'track'
        % If track is lost, fallback to other mode
        %PLACEHOLDER: ALLOW MISSED DETECTIONS VIA TRACKING
        if ~detected
            changed_mode = true;
            new_mode = mode_vars.track.fallback;
        end
end

%% Read Out Update

if readout
    if changed_mode
        fprintf('Switching from %s mode to %s mode.\n', current_mode, new_mode);
    else
        fprintf('Unit will remain in %s mode.\n', current_mode);
    end
end

%% Update Scenario Object To New Variables

if changed_mode
    
    % Set mode flag
    scenario.flags.mode = new_mode;
    
    % Update variables in radarsetup
    %PLACEHOLDER: Systematically update variables?
    scenario.radarsetup.int_type = mode_vars.(new_mode).int_type;
end

