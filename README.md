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

## Prerequisites
- A machine running Ubuntu 20.04 LTS or Ubuntu 22.04.
- Internet connection for downloading necessary software packages.

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

### PX4 Setup and Simulation

Follow these steps to install, run, and manage PX4 simulations.

```bash
# 1. Installing PX4

# Clone the PX4 repository
git clone https://github.com/PX4/PX4-Autopilot.git --recursive

# Run the setup script for Ubuntu
bash ./PX4-Autopilot/Tools/setup/ubuntu.sh

# Navigate into the PX4 directory
cd PX4-Autopilot/

# 2. Running PX4

# Single instance
make px4_sitl jmavsim

# Multiple instances
./Tools/simulation/sitl_multiple_run.sh 2

# 3. Simulation parameters

# Running jmavsim_run.sh with custom options
# -p <port> specifies the UDP port for JMAVSim-PX4 communication
# -l enables lockstep mode (synchronized step-by-step simulation)

# Example: two instances in lockstep mode
./Tools/simulation/jmavsim/jmavsim_run.sh -p 4560 -l
./Tools/simulation/jmavsim/jmavsim_run.sh -p 4561 -l

```
---
### Java Verification/Installation

Verify Java is installed. The output should say openjdk version "xx...

```bash
java -version

# 1. If Java is not installed, then it should say Command ‘java’ not found
Install OpenJDK 18:

sudo apt-get install openjdk-18-jdk
sudo apt-get install openjdk-18-jre

# 2. Verify the Java installation. The output should be similar to openjdk version "xx...:

java -version

```
### QGruoundControl Installation

1. Download QGroundControl: Download QGroundControl

2. Navigate to the directory where QGroundControl was downloaded (assuming it's the Downloads directory):

```bash
cd ~/Downloads
```

3. Make the QGroundControl.AppImage executable:

```bash
chmod +x ./QGroundControl.AppImage
```

4. Run QGroundControl:

```bash
./QGroundControl.AppImage
```
---

### Helm Installation

```bash
curl -fsSL https://get.helm.sh/helm-v3.15.2-linux-amd64.tar.gz -o helm.tgz
tar -xzf helm.tgz
sudo mv linux-amd64/helm /usr/local/bin/helm
helm version
```
---

## Creating robotcube namespace

```bash
kubectl create namespace robotkube
kubectl config set-context --current --namespace=robotkube
```

---

## How to Start

1. To start everything is enough to clone the repository inside the machine and launch the following command:

```bash
k3d cluster start robotcube
```
2. To stop everything we can use the following command :

```bash
k3d cluster stop
```



