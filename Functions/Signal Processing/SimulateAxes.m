function [cube] = SimulateAxes(scenario)
%SIMULATEAXES Generates range and doppler axes for simulated results

%% Unpack Variables

radarsetup = scenario.radarsetup;
sim = scenario.sim;

%% Define Constants

c = physconst('LightSpeed');
lambda = c/radarsetup.f_c;

%% Derive Axes

% Calculate FFT sizes
N_r = radarsetup.f_s * radarsetup.pri;
N_d = 2^ceil(log2(radarsetup.n_p));

% Derive Range axis
cube.range_res = (c/2)/radarsetup.f_s;
cube.sample_offset = radarsetup.t_p * radarsetup.f_s;
cube.range_offset = cube.sample_offset * cube.range_res;
cube.range_axis = (0:(N_r-1)) * cube.range_res;

% Derive Doppler axis
cube.vel_res = lambda*(radarsetup.prf)/(2*radarsetup.n_p);
cube.vel_axis = -cube.vel_res*((-N_d/2):(N_d/2));

end

