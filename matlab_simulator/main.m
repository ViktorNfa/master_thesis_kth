%% 2D multi-robot simulation for the master thesis titled:
% Control barrier function-enabled human-in-the-loop control for 
% multi-robot systems. By Victor Nan Fernandez-Ayala (vnfa@kth.se)

clear; clc; close all;

%% Parameter setup

% Arena size
x_max = 50;
y_max = 50;

% Robot size/diameter (modelled as a circle with a directional arrow)
d_robot = 1;

% Frequency of update of the simulation (in Hz)
freq = 50;

% Maximum time of the simulation (in seconds)
max_T = 30;

% Ideal formation positions
formation_positions = [[0, 2]; [0, 0]; [0, -2]; [2, 2]; [2, -2]];

% List of neighbours for each robot
neighbours = {[2]; [1, 3, 4, 5]; [2]; [2]; [2]};

% CBF Communication maintenance or obstacle avoidance activation 
% (1 is activated/0 is deactivated)
cbf_cm = 1;
cbf_oa = 1;

% Safe distance for communication maintenance and obstacle avoidance
safe_distance_cm = 3;
safe_distance_oa = 1.1;

% Variable to determine if HuIL is active or not 
% (1 is activated/0 is deactivated) as well as the robot it affects
huil = 1;
human_robot = 5;

%% Pre-calculations needed for controller and simulation

% Get graph Laplacian


%% Simulation and visualization loop

% Initialize position and velocities


% Start simulation loop
for i = 1:1/freq:max_T
    
    % Compute nominal controller
    u_nom = formationController(L, p, pd);
    
end

