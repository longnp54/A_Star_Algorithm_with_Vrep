# A Star Algorithm with V-REP/CoppeliaSim Integration

This project demonstrates the implementation of the A* algorithm for pathfinding, with MATLAB used for calculations and V-REP (CoppeliaSim) for simulation. The robot navigates through an environment, avoiding obstacles and following the optimal path calculated by the algorithm.

[**Demo Video**](https://drive.google.com/drive/folders/1KlpSVNjP1j9Ipvsl6B9JJJ24PooCiyEo?usp=drive_link)  
*Click the link above to watch the demonstration video hosted on Google Drive.*

---

## Table of Contents

1. [Overview](#overview)
2. [Features](#features)
3. [Requirements](#requirements)
4. [Installation and Setup](#installation-and-setup)
5. [How to Run](#how-to-run)
6. [Simulation Demonstrations](#simulation-demonstrations)
7. [Remote API Setup](#remote-api-setup)
8. [Acknowledgements](#acknowledgements)

---

## Overview

The A* algorithm is implemented in MATLAB to calculate the shortest path on a grid-based map. The calculated path is then passed to V-REP/CoppeliaSim using the Remote API, where a robot navigates the environment accordingly. This project showcases how robotics simulations can be integrated with algorithmic pathfinding solutions.

---

## Features

- **Pathfinding**: Optimal path calculation using the A* algorithm.
- **Obstacle Avoidance**: Dynamic grid map with obstacles.
- **Simulation**: Real-time robot movement simulation in V-REP/CoppeliaSim.

---

## Requirements

1. **MATLAB** (R2018a or later recommended)
2. **CoppeliaSim (formerly V-REP)**
3. **Remote API for MATLAB**

---

## Installation and Setup

### 1. Clone the Repository

```bash
git clone https://github.com/longnp54/A_Star_Algorithm_with_Vrep.git
cd A_Star_Algorithm_with_Vrep
```

### 2. Configure MATLAB

Ensure MATLAB is properly installed, and add the project directory to MATLAB's path.

### 3. Install CoppeliaSim

Download and install CoppeliaSim from the [official website](https://www.coppeliarobotics.com/).

### 4. Enable Remote API in CoppeliaSim

Enable the remote API in CoppeliaSim by adding the required scripts to your simulation project.

---

## How to Run

1. Open MATLAB and run the `A_star_pathfinding.m` script to calculate the optimal path.
2. Start the CoppeliaSim simulation.
3. Use the Remote API to transfer the path data to CoppeliaSim.
4. Observe the robot navigating the calculated path in the simulation environment.

---

## Simulation Demonstrations

### 1. **Robot in Action**

This is a snapshot of the robot navigating the simulation environment:

![Robot in Project](images/image_robot.png)

### 2. **MATLAB Pathfinding Map**

The following image shows the MATLAB-generated map with the grid, obstacles, and the optimal path highlighted:

![MATLAB Map](images/image_map_matlab.png)

### 3. **CoppeliaSim Simulation**

A screenshot from CoppeliaSim showcasing the robot following the optimal path:

![CoppeliaSim Simulation](images/image_map_vrep.png)

---

## Remote API Setup

To link MATLAB with CoppeliaSim using the Remote API, follow these steps:

1. **Include the Remote API Library**
   
   Copy the Remote API files (`remoteApi.dll`, `remoteApiProto.h`, etc.) into your project directory. Ensure MATLAB can access these files.

2. **Modify the CoppeliaSim Scene**

   - Open your scene in CoppeliaSim.
   - Add a "non-threaded child script" to your robot model.
   - Include the necessary API initialization commands in the script.

3. **Set Up the Remote Connection**

   - In MATLAB, initialize the Remote API using the following commands:
     
     ```matlab
     sim=remApi('remoteApi');
     clientID=sim.simxStart('127.0.0.1',19999,true,true,5000,5);
     if clientID > -1
         disp('Connected to CoppeliaSim!');
     else
         disp('Failed to connect.');
     end
     ```

4. **Run the Simulation**

   - Ensure CoppeliaSim is running.
   - Execute the MATLAB script to send the path data and control the robot.

---

## Acknowledgements

I would like to extend my heartfelt thanks to **Hung Pham** for his invaluable assistance in building the robot. Your support and guidance have been instrumental in bringing this project to life.

---

## License

This project is open-source and available for all


