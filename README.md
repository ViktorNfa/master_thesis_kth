# Master Thesis by Victor Nan Fernandez-Ayala (vnfa@kth.se)
Done as part of my MSc in Aerospace Engineering at KTH Royal Institute of Technology

# Control barrier function-enabled human-in-the-loop control for multi-robot systems
Supervised by Xiao Tan

## ROS Packages/Tasks list

### 1. [Formation with communication maintenance and/or obstacle avoidance](location of documentation) 
Use a multi-robot formation using as many robots as wanted to form whatever formation with whatever neighbour configuration while keeping a minimum distance between them to secure communication and prevent collisions. Additionally, one of the robots will include a Human-In-The-Loop controlling it at an specific timeframe as well as many other possible safety constraints that have been explored. Done both in a centralized and distributed way, as well as using Gazebo for the simulated version and the SML for the IRL version.

### 2. [Multi-agent python simulator](location of documentation) 
A Python-based simulator for 2D multi-robot systems created to simulate the algorithms for high numbers of agents in both a centralized and distributed way.

### 3. [Multi-agent distributed MATLAB simulator](location of documentation) 
A MATLAB-based simulator for 2D multi-robot systems created to simulate the algorithms for high numbers of agents in a distributed way.

<!---
### 3. [STL Platooning](location of documentation) 
Using STL grammar and the previously explored centralized CBF algorithms to define temporal tasks related to platooning. Two platoons will merge on one line while advancing in a pre-determined direction with a speed defined by HuIL.

### 4. [Satellite constellation control and visibility constraint](location of documentation) 
Using a multi-agent system representing a satellite constellation, a custom controller was created based on a nominal controller -in charge of maximizing the coverage around the Earth for each satellite- and a safety constraint -based on the visibility of each pair satellites, so that the Earth is not blocking them. This is simulated using a MATLAB simulator.
-->
