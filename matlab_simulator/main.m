%% 2D multi-robot simulation for the master thesis titled:
% Control barrier function-enabled human-in-the-loop control for 
% multi-robot systems. By Victor Nan Fernandez-Ayala (vnfa@kth.se)

clear; clc; close all;

%% Parameter setup

% Dimensionality of the problem
dim = 2;

% Arena size
x_max = 20;
y_max = 20;

% Robot size/diameter (modelled as a circle with a directional arrow)
d_robot = 1;

% Frequency of update of the simulation (in Hz)
freq = 50;

% Maximum time of the simulation (in seconds)
max_T = 30;

% Ideal formation positions
formation_positions = [[0, 2]; [0, 0]; [0, -2]; [2, 2]; [2, -2]];

% List of neighbours for each robot
neighbours = {[2, 4]; [1, 3, 4, 5]; [2, 5]; [1, 2]; [2, 3]};

% CBF Communication maintenance or obstacle avoidance activation 
% (1 is activated/0 is deactivated)
cm = 1;
oa = 1;

% Safe distance for communication maintenance and obstacle avoidance
d_cm = 2.75;
d_oa = 1.5;

% Variable to determine if HuIL is active or not 
% (1 is activated/0 is deactivated) as well as the robot it affects
huil = 1;
human_robot = 5;

%% Pre-calculations needed for controller and simulation

% Modify ideal formation positions to one column vector
pd = reshape(formation_positions',[],1);

% Get the number of robots
number_robots = length(neighbours);

number_edges = 0;
% Get graph Laplacian and number of edges
L = zeros(number_robots,number_robots);
for i=1:number_robots
    L(i,i) = length(neighbours{i});
    number_edges = number_edges + length(neighbours{i});
    for j=1:length(neighbours{i})
        L(i,neighbours{i})=-1;
    end
end
number_edges = number_edges/2;

% Create edge list
edges = zeros(number_edges,2);
e = 1;
for i=1:length(neighbours)
    for j = 1:length(neighbours{i})
        if i < neighbours{i}(j)
            edges(e,1) = i;
            edges(e,2) = neighbours{i}(j);
            e = e+1;
        end
    end
end
if e ~= number_edges+1
    fprint("An error creating the list of edges happened");
end


%% Simulation and visualization loop

max_time_size = max_T/(1/freq);

% Initialize position
p = x_max*2*rand(number_robots*dim,1)-x_max;

% Initialize visualization
for j = 1:number_robots
    plot(p(2*j-1),p(2*j),'or','MarkerSize',5,'MarkerFaceColor','r');
    grid on;
    hold on;
end
axis([-x_max x_max -y_max y_max])
title("Arena, t="+"0.00");
hold off;

% Start simulation loop
for i = 1:max_time_size
    
    % Compute nominal controller
    u_nom = formationController(L, p, pd);
    
    % Add HuIL controll
    u = huilController(u_nom, huil, human_robot, i, max_time_size);
    
    % Compute CBF constrained controller
    u = cbfController(p, u, cm, oa, d_cm, d_oa, edges);
    
    % Update the system using dynamics
    pdot = systemDynamics(p, u);
    p = pdot*(1/freq) + p;
    
    % Update visualization graph
    for j = 1:number_robots
        plot(p(2*j-1),p(2*j),'or','MarkerSize',5,'MarkerFaceColor','r');
        grid on;
        hold on;
    end
    axis([-x_max x_max -y_max y_max]);
    title(sprintf("Arena, t=%.*f", 2, i*1/freq));
    hold off;
    pause(1/freq);
    
end

