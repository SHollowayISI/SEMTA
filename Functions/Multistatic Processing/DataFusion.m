function [tracking_multi] = DataFusion(scenario)
%DATAFUSION Combination of data between multiple radar sensors
%   Performs inverse-variance weighted average of sensor positions and
%   velocity

%% Unpack variables

ts = scenario.tracking_single;
multi = scenario.multi;
tracking_multi = struct( ...
    'num_detect',   zeros(multi.n_fr, 1), ...
    'state',        [], ...
    'var',          [], ...
    'hit_list',     false(multi.n_fr, 1));
tracking_multi.state = cell(multi.n_fr, 1);
tracking_multi.var = cell(multi.n_fr, 1);

%% Perform data fusion

% Loop through frames
for fr = 1:multi.n_fr
    
    % Initialize sums
    num_detect = 0;
    state_sum = 0;
    var_sum = 0;
    
    % Loop through receivers
    for re = 1:multi.n_re
        
        % Only add if receiver detected target
        if ts{re}.hit_list(fr)
            
            % Unpack variables
            est = ts{re}.estimate{fr};
            state = est.state;
            var = diag(est.covar);
            
            % Adjust for unit position
            state(1) = state(1) + multi.radar_pos(1,re);
            state(3) = state(3) + multi.radar_pos(2,re);
            
            % Update running sums
            num_detect = num_detect + 1;
            state_sum = state_sum + (state ./ var);
            var_sum = var_sum + (1 ./ var);
            
        end
    end
    
    % If some data is collected
    if num_detect > 0
        
        % Calculate results per frame
        tracking_multi.num_detect(fr) = num_detect;
        tracking_multi.state{fr} = state_sum ./ var_sum;
        tracking_multi.var{fr} = diag(1 ./ var_sum);
        tracking_multi.hit_list(fr) = true;
        
    end
    
end

end

