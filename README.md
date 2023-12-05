# Master Thesis by Victor Nan Fernandez-Ayala (vnfa@kth.se)
Done as part of my MSc in Aerospace Engineering at KTH Royal Institute of Technology

# Control barrier function-enabled human-in-the-loop control for multi-robot systems
Supervised by Xiao Tan

## Installation
Clone the repository in your desired directory with `git clone https://github.com/KTH-DHSG/cbf-enabled-hil-control-mas.git`.

## ROS Packages & Other folders

### 1. [Formation with communication maintenance and/or obstacle avoidance](first_task_formation_cbf) 
This package can be found in the first_task_formation_cbf folder and requires a basic installation of ROS 1 in a Linux environment.

Description: Use a multi-robot formation using as many robots as wanted to form whatever formation with whatever neighbour configuration while keeping a minimum distance between them to secure communication and prevent collisions. Additionally, one of the robots will include a Human-In-The-Loop controlling it at an specific timeframe as well as many other possible safety constraints that have been explored during the thesis. Done both in a centralized and distributed way, using Gazebo for the simulated version and the SML for the IRL version.

### 2. [Multi-agent python simulator](python_simulator) 
This package can be found in the python_simulator folder and requires a basic installation of Python with the scipy, numpy, matplotlib, pandas and tqdm python packages.

Description: A Python-based simulator for 2D multi-robot systems created to simulate the algorithms for high numbers of agents in both a centralized and distributed way.

### 3. [Multi-agent distributed MATLAB simulator](DCBF_matlab_code) 
This package can be found in the DCBF_matlab_code folder and requires a basic installation of MATLAB.

Description: A MATLAB-based simulator for 2D multi-robot systems created to simulate the algorithms for high numbers of agents in a distributed way.

### 4. [General documentation and Master Thesis](documentation) 
This folder contains the thesis as well as some data and plots from the Gazebo simulations. The rest of the videos can be found on the YouTube videos linked in the pdf file of the thesis.

### 5. [Auxiliary code](auxiliary_code) 
This folder contains auxiliary code to plot the cbf functions and controller output values of the experiments.