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
t_rcs = db2pow(rcs_out.ave_rcs);        % Target RCS in m^2
lambda = c./rcs_out.freq;               % Vector of lambda values in m

%% Constant RCS Function

function value = staticRCS(rcs, ang, freq)
    value = db2pow(rcs.ave_rcs) * ones(length(ang),length(freq));
end

%% Fluctuating RCS Model Function

function value = modelRCS(rcs, ang, freq)
    value = interp2(rcs.freq_array, rcs.ang_array, rcs.rcs_array, freq, ang, 'nearest');
end

%% Run Model

% Set constant if desired
if strcmp(rcs_in.rcs_model, 'model')
    
    % Place Scatterers
    loc_s = (rand(3,rcs_out.n_sc)-0.5).*rcs_out.dim;
    
    % Pre-allocate Output Size
    total_rcs = zeros(length(rcs_out.ang),length(rcs_out.freq));
    
    % Generate Steering Vector
    for n = 1:length(lambda)
        
        % Generate steering vector per scatterer per frequency
        sv = steervec(loc_s/lambda(n), rcs_out.ang);
        
        % Sum RCS values given all point scatterers have RCS = 1 [m^2]
        % Steering vector is squared due to round trip length
        total_rcs(:,n) = transpose(abs(sum(sv.^2)).^2);
        
    end
    
    % Normalize to correct target average RCS value
    rcs_out.rcs_array = t_rcs * total_rcs ./ mean(total_rcs,2);
    
    % Generate arrays of frequencies and angles for interpolation coords
    [rcs_out.freq_array, rcs_out.ang_array] = meshgrid(rcs_out.freq, rcs_out.ang);
    
end

%% Attach Model Function

switch rcs_in.rcs_model
    case 'model'
        rcs_out.evaluate = @modelRCS;
    case 'constant'
        rcs_out.evaluate = @staticRCS;
end


end

