function [cube] = SignalProcessing(scenario)
%SIGNALPROCESSING_SEMTA Performs signal processing for SEMTA
%   Takes scenario struct as input, retuns scenario.cube struct containing
%   processed Range-Doppler cube

%% Unpack Variables

radarsetup = scenario.radarsetup;
sim = scenario.sim;

%% Define Constants

c = physconst('LightSpeed');
lambda = c/radarsetup.f_c;

%% Perform Range Matched Filter

% Calculate FFT size
N_r = size(scenario.rx_sig, 1);

% Perform correlation processing on received signal
cube.range_cube = ifft(fft(scenario.rx_sig, N_r, 1) .* conj(fft(sim.waveform(), N_r, 1)));

% Split into fast time x slow time x CPI
cube.range_cube = reshape(cube.range_cube, ...
    size(cube.range_cube,1), radarsetup.n_p, radarsetup.cpi_fr, 2);

%% Perform Doppler FFT

% Calculate FFT size
N_d = 2^ceil(log2(size(cube.range_cube,2)));

% Apply windowing
expression = '(size(cube.range_cube,2))).*cube.range_cube;';
expression = ['transpose(', radarsetup.win_type, expression];
cube.rd_cube = eval(expression);

% FFT across slow time dimension
cube.rd_cube = fftshift(fft(cube.rd_cube, N_d, 2), 2);

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
cube.range_axis = (0:(N_r-1)) * cube.range_res;

% Derive Doppler axis
cube.vel_res = lambda*(radarsetup.prf)/(2*radarsetup.n_p);
cube.vel_axis = -cube.vel_res*((-N_d/2):(N_d/2));

end

