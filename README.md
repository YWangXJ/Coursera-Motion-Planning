# Coursera-Motion-Planning
This repository contains all code assignments for Course: Motion Planning for Self-Driving Cars 

## Capstone
The Capstone project (Folder "*Course4FinalProject*") aims at having a functional motion planning stack that can avoid both static and dynamic obstacles while tracking the center line of a lane, while also handling stop signs. The final output is tested in Autonomous Vehicle Simulator [CARLA](https://carla.org/). 

Modules includes:
1. Behavioral Planning through Finte State Machine
2. Static Collision Checking through 
3. Path Selection
4. Velocity Profile Generation

### Behaviour Planning (behavioural_planner.py)
The behavioural logic in this project required to handle a stop sign. A state machine that transitions between lane following, deceleration to the stop sign, staying stopped, and back to lane following, when it encounters a stop sign. 

## Occupancy Grid Generation
Folder "*OccupancyGrid*" contains the python code for  Occupancy Grid Generation. The code 
    "* Gather range measurements of a moving car's surroundings using a lidar scanning function.\n",
    "* Extract occupancy information from the range measurements using an inverse scanner model.\n",
    "* Perform logodds updates on an occupancy grids based on incoming measurements.\n",
    "* Iteratively construct a probabilistic occupancy grid from those log odds updates.\n",
    "\n",
## Path Planning
### Dijkstra

### A_star


