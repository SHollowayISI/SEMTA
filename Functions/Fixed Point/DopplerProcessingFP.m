function [y] = DopplerProcessingFP(x)
%DOPPLERPROCESSINGFP Fixed point doppler processing
%   Takes single pulse response as input, returns correlation signal as
%   output.

%% Unpack Variables

radarsetup = scenario.radarsetup;
sim = scenario.sim;

%% Define Constants

c = physconst('LightSpeed');
lambda = c/radarsetup.f_c;

%% Perform Range Correlation

% Calculate FFT size
N_r = size(scenario.rx_sig, 1);

% Perform initial FFT
cube.range_cube = fft(scenario.rx_sig, N_r, 1);

% Apply windowing to reference signal
if ~strcmp(radarsetup.r_win, 'none')
    
    % Calculate samples in chirp time
    N_ch = floor(radarsetup.t_p * radarsetup.f_s);
    
    % Generate windowing
    window = [];
    eval(['window = ', radarsetup.r_win, '(N_ch);']);
    window = [window; ones(N_r - N_ch, 1)];
    
    % Generate reference signal
    ref_sig = conj(fft(window .* sim.waveform(), N_r, 1));
else
    ref_sig = conj(fft(sim.waveform(), N_r, 1));
end

% Perform correlation processing on FFT of received signal
cube.range_cube = ifft(cube.range_cube .* ref_sig);

% Split into fast time x slow time x CPI
cube.range_cube = reshape(cube.range_cube, ...
    size(cube.range_cube,1), radarsetup.n_p, radarsetup.cpi_fr, 2);

%% Perform Doppler FFT

% Calculate FFT size
N_d = 2^ceil(log2(size(cube.range_cube,2)));

% Apply windowing
if strcmp(radarsetup.d_win, 'none')
    cube.rd_cube = cube.range_cube;
else
    expression = '(size(cube.range_cube,2))).*cube.range_cube;';
    expression = ['transpose(', radarsetup.d_win, expression];
    cube.rd_cube = eval(expression);
end

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

