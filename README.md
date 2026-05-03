# 4-DOF Fin Ray Robot Arm

A self-designed and 3D-printed 4-DOF robotic arm with compliant fin-ray-style joints, ROS 2 Jazzy control, Damped Least Squares inverse kinematics, and RViz2 visualization.

![demo](./ik_4dof.gif)

## Overview

This project implements a custom 4-DOF robotic arm from mechanical design to ROS 2-based motion control.

All mechanical parts were designed from scratch in **Fusion 360** and fabricated using **3D printing**.  
The robot uses **Dynamixel XL330** actuators and is intended to be controlled through an **OpenRB-150** controller.

The current software stack includes URDF modeling, RViz2 visualization, target pose command input, and inverse kinematics using **Damped Least Squares (DLS)**.

## Status

| Component | Status |
|---|---|
| Mechanical design | Complete |
| 3D-printed structure | Complete |
| URDF modeling | Complete |
| RViz2 visualization | Complete |
| DLS inverse kinematics | Complete |
| ROS 2 topic-based control simulation | Complete |
| PC ↔ OpenRB-150 / ESP32 hardware communication | In progress |
| Full hardware actuation test | In progress |

## Hardware

| Part | Description |
|---|---|
| Actuators | Dynamixel XL330 ×4 |
| Controller | OpenRB-150 |
| Structure | Custom compliant fin-ray-style joints |
| Fabrication | 3D-printed parts |
| Design Tool | Fusion 360 |

## Software Stack

- **ROS 2 Jazzy**
- **RViz2**
- **URDF**
- **Python / C++**
- **Damped Least Squares (DLS) IK**
- **Dynamixel / OpenRB-150 hardware integration** *(in progress)*

## System Architecture

| Node | Subscribes | Publishes | Description |
|---|---|---|---|
| `my_arm_ik_node` | `/tip_target` | `/desired_joint_states` | Solves inverse kinematics using DLS |
| `my_arm_seq_controller_node` | `/desired_joint_states` | `/joint_states` | Sequential joint controller for simulation |
| `robot_state_publisher` | `/joint_states` | TF | Publishes URDF-based transform tree |

## Control Flow

```text
/tip_target
    ↓
my_arm_ik_node
    ↓
/desired_joint_states
    ↓
my_arm_seq_controller_node
    ↓
/joint_states
    ↓
robot_state_publisher
    ↓
RViz2 visualization
