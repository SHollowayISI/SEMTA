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
t_st = radarsetup.n_p * radarsetup.cpi_fr * (flags.frame - 1);
    
% Set radar position and velocity
tx_pos = multi.radar_pos(:,flags.unit);
tx_vel = [0; 0; 0];


%% Simulation

% Generate the pulse
tx_sig = sim.waveform();

% Transmit the pulse. Output transmitter status
[tx_sig,tx_status] = sim.transmitter(tx_sig);

for chirp = 1:radarsetup.n_p
    
    % Update the target position
%     tgt_pos = traj.pos(:, chirp + t_st);
%     tgt_vel = traj.vel(:, chirp + t_st);
    
    %PLACEHOLDER
    tgt_pos = [0; 0; 0];
    tgt_vel = [0; 0; 0];

    % Get the range and angle to the target
    [~,tgt_ang] = rangeangle(tgt_pos,tx_pos);
    
    % Steer beam towards target
    if ((chirp == 1) && (radarsetup.n_ant > 1))
        steering_angle = -1*tgt_ang;
        sv = steervec(getElementPosition(sim.sub_array)/lambda, steering_angle(1));
        sim.sub_array.Taper = sv;
        
        sim.radiator = phased.Radiator( ...
            'Sensor',                   sim.array, ...
            'PropagationSpeed',         c, ...
            'OperatingFrequency',       radarsetup.f_c, ...
            'CombineRadiatedSignals',   true);
        
        sim.collector = phased.Collector( ...
            'Sensor',                   sim.array, ...
            'PropagationSpeed',         c, ...
            'OperatingFrequency',       radarsetup.f_c, ...
            'Wavefront',                'Plane');
    end
    
    % Radiate the pulse toward the target
    sig = sim.radiator(tx_sig,tgt_ang);
    
    % Propagate the pulse to the target and back in free space
    sig = sim.target_chan(sig,tx_pos,tgt_pos,tx_vel,tgt_vel);
    
    % Reflect the pulse off the target
    ang_idx = round(tgt_ang(2)/rcs.res_a)+(length(rcs.ang)+1)/2;
    sig = sim.target(sig, rcs.value(ang_idx));
    
    % Collect the echo from the incident angle at the antenna
    sig = sim.collector(sig,tgt_ang);
    
    % Reshape signal to fast time x slow time
    sig = sim.receiver(sig, ~tx_status);
    
    % Receive the echo at the antenna when not transmitting
    rx_sig(:,chirp,:) = sig;
    
    
end

scenario_out.sim = sim;
scenario_out.rx_sig = rx_sig;

end

