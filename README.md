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

## Requirements

Before starting, make sure you have the following tools installed:

| Tool | Version | Description |
|------|----------|-------------|
| [Docker](https://www.docker.com/) | ≥ 24.0 | Container runtime |
| [K3d](https://k3d.io/) | ≥ 5.6 | Lightweight Kubernetes cluster in Docker |
| [kubectl](https://kubernetes.io/docs/tasks/tools/) | ≥ 1.28 | Kubernetes CLI |
| [ROS 2 Humble](https://docs.ros.org/en/humble/index.html) | Recommended | Middleware for robotics |
| [PX4 Autopilot](https://px4.io/) | ≥ 1.14 | UAV autopilot software |
| [QGroundControl](https://qgroundcontrol.com/) | latest | Ground Control Station GUI |
| [MongoDB](https://www.mongodb.com/) | ≥ 6.0 | Database for UAV data |

---

## How It Works

1. **PX4 SITL** simulates UAVs and publishes telemetry via **MAVLink**.  
2. **Micro XRCE-DDS Agent** bridges MAVLink messages to ROS 2 topics.  
3. The **Event Detector** node monitors telemetry and detects conditions (e.g., distance < 50 m).  
4. Upon detection, it triggers **RobotKube**, which dynamically deploys or removes applications (e.g., a Data Recorder) inside the **K3d cluster**.  
5. The **Data Recorder** writes the UAV data into **MongoDB**.  
6. **QGroundControl** allows visualization and manual mission control.

---

## Setup

### 1. Installing Px4

At the start we did all the necessary steps to install correctly the PX4 library.

```bash
git clone https://github.com/PX4/PX4-Autopilot.git --recursive
bash ./PX4-Autopilot/Tools/setup/ubuntu.sh
cd PX4-Autopilot/

To run just a single instance of PX4 is used the command :

```bash
make px4_sitl jmavsim

To run two different instances of PX4 we can use the following command:

```bash
./Tools/simulation./sitl_multiple_run.sh 2

In the simulation setup, two important parameters are passed to the script jmavsim run.sh. The
option -p is used to specify the UDP port through which JMAVSim communicates with PX4.
For instance, launching the simulator with -p 4560 establishes the connection on port 4560. This
mechanism allows multiple instances of JMAVSim to run in parallel, as each instance can be assigned
a different communication port, thereby avoiding conflicts. The option -l, instead, enables the
so-called lockstep mode. When this mode is active, JMAVSim and PX4 progress in a strictly
synchronized fashion: the simulator advances the physical model step by step, waiting for PX4
to complete the processing of each step before continuing.

```bash
./Tools/simulation/jmavsim/jmavsim_run.sh -p 4560 -l
./Tools/simulation/jmavsim/jmavsim_run.sh -p 4561 -l


