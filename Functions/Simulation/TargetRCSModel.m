function [rcs_out] = TargetRCSModel(rcs_in)
%TARGETRCSMODEL Models complex target RCS for TRASAT-SEMTA
%   Takes as input scenario.rcs struct, outputs scenario.rcs struct with
%   modified 'value' and 'ang' fields

%% Set up variables

% Transfer values
rcs_out = rcs_in;

% Physical Constants
c = physconst('LightSpeed');            % Speed of light in m/s

% Derived Variables
rcs_out.ang = -180:rcs_out.res_a:180;   % Vector of aspect angles in degree
t_rcs = 10^(rcs_out.ave_rcs/10);        % Target RCS in m^2
lam = c./rcs_out.freq;                  % Vector of lambda values in m

%% Run Model

% Set constant if desired
if strcmp(rcs_in.rcs_model, 'constant')
    rcs_out.value = t_rcs*ones(length(rcs_out.ang),length(rcs_out.freq));
else
    
    % Place Scatterers
    loc_s = (rand(3,rcs_out.n_sc)-0.5).*rcs_out.dim;
    
    % Pre-allocate Output Size
    total_rcs = zeros(length(rcs_out.ang),length(rcs_out.freq));
    
    % Generate Steering Vector
    for n = 1:length(lam)
        
        % Generate steering vector per scatterer per frequency
        sv = steervec(loc_s/lam(n), rcs_out.ang);
        
        % Sum RCS values given all point scatterers have RCS = 1 [m^2]
        % Steering vector is squared due to round trip length
        total_rcs(:,n) = transpose(abs(sum(sv.^2)).^2);
        
    end
    
    % Normalize to correct target average RCS value
    rcs_out.value = t_rcs * total_rcs ./ mean(total_rcs,2);
    % rcs_out.value = 10*log10(total_rcs);
end


end

