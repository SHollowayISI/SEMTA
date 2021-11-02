function [cube] = SignalProcessing_FixedPoint(scenario)
%SIGNALPROCESSING_FIXEDPOINT Performs signal processing for SEMTA
%   Takes scenario struct as input, retuns scenario.cube struct containing
%   processed Range-Doppler cube

%% Unpack Variables

radarsetup = scenario.radarsetup;

%% Define Constants

c = physconst('LightSpeed');
lambda = c/radarsetup.f_c;

%% Perform Range-Doppler Processing

% Load reference function
filein = load('Fixed Point\Reference\ReferenceSignal.mat');
x_ref = filein.x_ref;

% Split cube by CPI
for cpi = 1:radarsetup.cpi_fr

    % Obtain single CPI cube
    ch_ind = ((cpi-1)*radarsetup.n_p + 1):(cpi*radarsetup.n_p);
    x = scenario.rx_sig(:,ch_ind,:);
    
    % Perform range-Doppler processing in fixed-point
    y = RDProcessingFP_wrapper_fixpt_mex('RDProcessingFP_wrapper_fixpt', x, x_ref);
    
    % Assign result to data cube
    cube.rd_cube(:,:,cpi,:) = y;
    
end

% Wrap max negative frequency and positive frequency
cube.rd_cube = [cube.rd_cube, cube.rd_cube(:,1,:,:)];

%% Calculate Square Power Cube

% Take square magnitude of radar cube, averaged across CPI
cube.pow_cube = abs(cube.rd_cube).^2;

%% Derive Axes

% Derive Range axis
cube.range_res = (c/2)/radarsetup.f_s;
cube.sample_offset = radarsetup.t_p * radarsetup.f_s;
cube.range_offset = cube.sample_offset * cube.range_res;
cube.range_axis = (0:(1250-1)) * cube.range_res;

% Derive Doppler axis
cube.vel_res = lambda*(radarsetup.prf)/(2*radarsetup.n_p);
cube.vel_axis = cube.vel_res*((-1024/2):(1024/2));

end

