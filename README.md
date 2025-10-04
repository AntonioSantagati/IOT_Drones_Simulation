# IOT_Drones_Simulation
Drones Simulations using JmavSim and QGroundControl

#  Distributed End-to-End UAV Simulation with PX4, ROS 2, and RobotKube

This repository contains the implementation of a **distributed simulation framework** for UAV systems, integrating **PX4 SITL**, **ROS 2**, and **RobotKube** within a **K3d** Kubernetes cluster.  
The goal is to demonstrate how **event-driven orchestration** can dynamically manage application modules in response to real-time UAV telemetry.

---

## Overview

The system simulates multiple UAVs in a distributed environment, allowing **automatic deployment** of applications (e.g., data recording) based on the state of the simulation.  
When certain events occur—such as two drones flying within a specified distance—RobotKube dynamically triggers or stops modules according to defined rules.

### Key Features
- **PX4 SITL** for UAV simulation  
- **ROS 2** as communication middleware (DDS-based)  
- **RobotKube** for event-driven orchestration in Kubernetes  
- **MongoDB** for persistent UAV data storage  
- **QGroundControl** as ground control station  
- Fully containerized setup using **Docker** and **K3d**

---

##  System Architecture

