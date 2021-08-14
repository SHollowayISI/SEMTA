function [scenario_out] = RadarSimulation(scenario)
%RADARSIMULATION_SEMTA Generates simulated radar response for SEMTA
%   Takes scenario.sim, .radarsetup, .traj, .multi as inputs and outputs
%   scenario.rx_sig struct containing the received signal.

%% Unpack Variables

scenario_out = scenario;

sim = scenario.sim;
radarsetup = scenario.radarsetup;
traj = scenario.traj;
rcs = scenario.rcs;
flags = scenario.flags;
multi = scenario.multi;

%% Setup

% Physical constants
c = physconst('LightSpeed');

% Derived variables
lambda = c/radarsetup.f_c;
pri = 1/radarsetup.prf;
n_samples = radarsetup.f_s*pri;

% Allocate matrix for output signal
rx_sig = zeros(n_samples, radarsetup.n_p*radarsetup.cpi_fr, 2);

% Set up start time
t_st = radarsetup.frame_time * (flags.frame - 1);
    
% Set radar position and velocity
tx_pos = multi.radar_pos(:,flags.unit);
tx_vel = [0; 0; 0];


%% Start of Simulation Tasks

% Generate the pulse
tx_sig = sim.waveform();

% Transmit the pulse. Output transmitter status
[tx_sig,tx_status] = sim.transmitter(tx_sig);

% Calculate initial steering angle
st_ang = multi.steering_angle(flags.frame, flags.unit);

% Calculate steering vectors
Tx_steer = steervec(getElementPosition(sim.sub_array)/lambda, ...
    -1*st_ang, radarsetup.phase_bits);
Rx_steer = steervec(getElementPosition(sim.sub_array)/lambda, ...
    -1*[st_ang + radarsetup.beamwidth/2, st_ang - radarsetup.beamwidth/2], ...
    radarsetup.phase_bits);


%% Main Simulation Loop

% Loop over chirps
for chirp = 1:(radarsetup.n_p  * radarsetup.cpi_fr)

    % Calculate target position and velocity
    tgt_pos = traj.pos(traj, t_st + ((chirp-1) * radarsetup.pri));
    tgt_vel = traj.vel(traj, t_st + ((chirp-1) * radarsetup.pri));

    % Get the range and angle to the target
    [~,tgt_ang] = rangeangle(tgt_pos,tx_pos);
    
    % Radiate the pulse toward the target
    sig = sim.radiator(tx_sig, tgt_ang, Tx_steer);
    
    % Propagate the pulse to the target and back in free space
    sig = sim.target_chan(sig,tx_pos,tgt_pos,tx_vel,tgt_vel);
    
    % Reflect the pulse off the target
    sig = sim.target(sig, rcs.evaluate(rcs, tgt_ang(1), radarsetup.f_c));
    
    % Collect the echo from the incident angle at the antenna
    sig = sim.collector(sig, tgt_ang, Rx_steer);
    
    % Reshape signal to fast time x slow time
    sig = sim.receiver(sig, ~tx_status);
    
    % Receive the echo at the antenna when not transmitting
    rx_sig(:,chirp,:) = sig;
    
end

% Pack variables for output
scenario_out.sim = sim;
scenario_out.rx_sig = rx_sig;
scenario_out.multi = multi;

end

