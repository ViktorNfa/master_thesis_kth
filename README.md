# Master Thesis by Victor Nan Fernandez-Ayala (vnfa@kth.se)
Done as part of my MSc in Aerospace Engineering at KTH Royal Institute of Technology

# Control barrier function-enabled human-in-the-loop control for multi-robot systems
Supervised by Xiao Tan

## ROS Packages/Tasks list

### 1. [Formation with communication maintenance and/or obstacle avoidance](location of documentation) 
Use a multi-robot formation using as many robots as wanted to form whatever formation with whatever neighbour configuration while keeping a minimum distance between them to secure communication and prevent collisions. Additionally, one of the robots will include a Human-In-The-Loop controlling it at an specific timeframe as well as many other possible safety constraints that have been explored. Done both in a centralized and decentralized way, as well as using Gazebo for the simulated version and the SML for the IRL version.

A MATLAB simulator for 2D multi-robot systems was created to simulated the algorithm for high numbers of agents.

### 2. [Satellite constellation control and visibility constraint](location of documentation) 
Using a multi-agent system representing a satellite constellation, a custom controller was created based on a nominal controller -in charge of maximizing the coverage around the Earth for each satellite- and a safety constraint -based on the visibility of each pair satellites, so that the Earth is not blocking them. This is simulated using a MATLAB simulator.
