# Master Thesis by Victor Nan Fernandez-Ayala (vnfa@kth.se)
Done as part of my MSc in Aerospace Engineering at KTH Royal Institute of Technology

# Control barrier function-enabled human-in-the-loop control for multi-robot systems
Supervised by Xiao Tan

## ROS Packages/Tasks list

### 1. [Formation and communication maintenance](location of documentation) 
Use a multi-robot formation using as many robots as wanted (currently max. of 5 to ensure convergence) to form whatever formation with whatever neighbour configuration while keeping a minimum distance between them to secure communication and prevent collisions. Additionally, one of the robots will include a Human-In-The-Loop controlling it at an specific timeframe. Done both in a centralized and decentralized way (without assured convergence for now).
