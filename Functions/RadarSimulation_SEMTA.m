function [scenario_out] = RadarSimulation_SEMTA(scenario)
%RADARSIMULATION_SEMTA Generates simulated radar response for SEMTA
%   Takes scenario.sim, .simsetup, .traj, .multi as inputs and outputs
%   scenario.rx_sig struct containing the received signal.

%% Unpack Variables

scenario_out = scenario;

sim = scenario.sim;
simsetup = scenario.simsetup;
traj = scenario.traj;
rcs = scenario.rcs;
flags = scenario.flags;
multi = scenario.multi;

%% Setup

% Physical constants
c = physconst('LightSpeed');

% Derived variables
lambda = c/simsetup.f_c;
pri = 1/simsetup.prf;
n_samples = simsetup.f_s*pri;

% Allocate matrix for output signal
rx_sig = zeros(n_samples, simsetup.n_p*simsetup.cpi_fr);

% Set up start time
t_st = simsetup.n_p * simsetup.cpi_fr * (flags.frame - 1);
    
% Set radar position and velocity
tx_pos = multi.radar_pos(:,flags.unit);
tx_vel = [0; 0; 0];


%% Simulation

% Generate the pulse
tx_sig = sim.waveform();

% Transmit the pulse. Output transmitter status
[tx_sig,tx_status] = sim.transmitter(tx_sig);

for n = 1:(simsetup.n_p/flags.sim_rate)
    
    % Update the target position
    tgt_pos = traj.pos(:,(n-1)*flags.sim_rate + t_st + 1);
    tgt_vel = traj.vel(:,(n-1)*flags.sim_rate + t_st + 1);
    
    % Get the range and angle to the target
    [~,tgt_ang] = rangeangle(tgt_pos,tx_pos);
    
    % Steer beam towards target
    if ((n == 1) && (simsetup.n_ant > 1))
        steering_angle = -1*tgt_ang;
        sv = steervec(getElementPosition(sim.sub_array)/lambda, steering_angle(1));
        sim.sub_array.Taper = sv;
        
        sim.radiator = phased.Radiator( ...
            'Sensor',                   sim.array, ...
            'PropagationSpeed',         c, ...
            'OperatingFrequency',       simsetup.f_c, ...
            'CombineRadiatedSignals',   true);
        
        sim.collector = phased.Collector( ...
            'Sensor',                   sim.array, ...
            'PropagationSpeed',         c, ...
            'OperatingFrequency',       simsetup.f_c, ...
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
    sig = sim.receiver(sig(:,1), ~tx_status);
    
    % Receive the echo at the antenna when not transmitting
    rx_sig(:,((n-1)*flags.sim_rate + 1):(n*flags.sim_rate)) = ...
        reshape(sig, n_samples, flags.sim_rate);
    
    
end

scenario_out.sim = sim;
scenario_out.rx_sig = rx_sig;

end

