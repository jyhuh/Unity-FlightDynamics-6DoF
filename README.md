# FlightDynamics 6-DOF: Nonlinear UAV & eVTOL Flight Dynamics Suite for Unity 6

[![License: Apache-2.0](https://img.shields.io/badge/License-Apache%202.0-blue.svg)](LICENSE)
[![Unity 6 Ready](https://img.shields.io/badge/Unity-6000.3.11f1%20(Unity%206)-black.svg?logo=unity)](https://unity.com/)
[![Runtime: .NET Standard 2.1](https://img.shields.io/badge/.NET%20Standard-2.1-512BD4.svg?logo=dotnet)](https://dotnet.microsoft.com/)
[![PX4 Autopilot SITL](https://img.shields.io/badge/PX4-v1.15%20SITL-orange.svg?logo=drone)](https://px4.io/)
[![ROS 2](https://img.shields.io/badge/ROS%202-Humble%20Hawksbill-22314E.svg?logo=ros)](https://www.ros.org/)
[![Automated Assertions](https://img.shields.io/badge/Tests-196%20Passing-brightgreen.svg)](dotnet/FlightDynamics6DoF.Tests/)

**FlightDynamics 6-DOF** is a high-fidelity aerospace flight dynamics and simulation engineering suite for Unity. It combines offline aerodynamic polar generation (CST parametric airfoils, 3D Vortex Lattice Method, and CCBlade BEMT) with real-time multi-fidelity 6-DOF RK4 integration, zero-dependency MAVLink v2 lockstep communication, and a lightweight containerized PX4 Autopilot SITL stack optimized for macOS (Apple Silicon / Intel).

---

## Key Highlights

- **Multi-Fidelity 6-DOF Physics (Levels 0..3)**:
  - **Level 0**: Lightweight Unity `Rigidbody` coupled with lumped $K_t / K_q$ rotor thrust and bluff body drag.
  - **Level 1**: RK4 6-DOF simulation with pre-baked aerodynamic stability derivatives and surrogate BEMT curves.
  - **Level 2**: Strip-theory nonlinear lifting line solver with blade element rotor inflow dynamics.
  - **Level 3**: Full unsteady 3D Vortex Lattice Method (VLM) wake induction, rotor-slipstream wing impingement, and MIL-F-8785C Dryden turbulence.
- **Runway Ground Dynamics & Landing Gear Model**:
  - Coulomb rolling friction ($\mu_{\text{roll}} \approx 0.025$) for realistic fixed-wing runway acceleration.
  - Lateral tire cornering stiffness, nose-gear rotation freedom ($V_{\text{rot}}$), and automatic idle parking brake.
  - Surface boundary layer turbulence attenuation preventing parked airframe drift.
- **Zero-Dependency MAVLink v2 Lockstep & QGC Integration**:
  - Direct hardware-in-the-loop (HIL) lockstep (`HIL_SENSOR`, `HIL_ACTUATOR_CONTROLS`) over TCP port `4560`.
  - True dynamic pressure ($\frac{1}{2}\rho V^2$) airspeed telemetry for PX4 EKF2 wind estimation and TECS fixed-wing controllers.
  - Independent UDP telemetry stream (`UDP :14550`) for simultaneous QGroundControl mission monitoring.
- **Containerized macOS SITL Stack**:
  - Native container management via Colima (`containerd` + `vz` + `virtiofs`) and Apple `container` CLI.
  - Complete in-editor launcher GUI (`Px4SitlProcessLauncherEditor`) for starting, stopping, and inspecting PX4 SITL containers with zero shell friction.
- **ROS 2 Humble & Micro-XRCE-DDS Support**:
  - Native uORB topic bridging over UDP port `8888` (`/fmu/in/*` and `/fmu/out/*`).
  - Pre-configured Python offboard trajectory controller for autonomous mission execution.
- **Clean-Room Engineering & Permissive Licensing**:
  - 100% free of GPL/LGPL contamination. Pure Apache 2.0 license.

---

## System Architecture

FlightDynamics separates **Stage 1 (Pre-Simulation Aerodynamic Analysis)** from **Stage 2 (Real-Time 6-DOF Simulation)**:

```mermaid
flowchart TD
    subgraph Stage1["Stage 1: Pre-Flight Aerodynamic Analysis & Polar Baking"]
        A1["Parametric Aircraft Spec<br/>(Wings, Rotors, Fuselage, Control Surfaces)"]
        M1["Module 1: 2D Airfoil Polar Solver<br/>(CST Airfoils + Viterna ±180° + Du-Selig 3D Stall)"]
        M2["Module 2: 3D Vortex Lattice Method (VLM)<br/>(Trefftz Plane Downwash & Control Surface Ineffectiveness)"]
        M3["Module 3: Fuselage & Parasitic Solver<br/>(Slender Body Theory + Cross-flow Drag)"]
        M4["Module 4: CCBlade BEMT Solver<br/>(Prandtl Hub/Tip Losses, High-Thrust Glauert, Slipstream)"]
        M5["Module 5: Stability & Trim Analyzer<br/>(Longitudinal/Lateral Derivatives, Neutral Point, Mixer Matrix)"]
        DB[("BakedAeroDatabase")]
        A1 --> M1 & M2 & M3 & M4 --> M5 --> DB
    end

    subgraph Stage2["Stage 2: Multi-Fidelity Real-Time RK4 Simulation Engine"]
        L0["Level 0: Unity Rigidbody<br/>(Lumped Forces & Bluff Drag)"]
        L1["Level 1: 6-DOF Surrogate<br/>(RK4 + Baked Derivatives & Curves)"]
        L2["Level 2: 6-DOF Blade & Strip<br/>(RK4 + Real-Time BEMT & Strip Lift)"]
        L3["Level 3: Full 6-DOF Aero Wake<br/>(RK4 + Real-Time 3D VLM + Rotor Wake + Dryden)"]
        DB --> L0 & L1 & L2 & L3
    end

    subgraph HardwareLoop["Autopilot & Ground Station Telemetry"]
        PX4["PX4 SITL Container<br/>(v1.15 Lockstep :4560)"]
        QGC["QGroundControl GCS<br/>(UDP :14550)"]
        ROS2["ROS 2 Humble<br/>(Micro-XRCE-DDS :8888)"]
        L0 & L1 & L2 & L3 <-->|"MAVLink v2 Lockstep"| PX4
        PX4 <-->|"Telemetry"| QGC
        PX4 <-->|"uORB Topics"| ROS2
    end
```

---

## Supported Vehicle Configurations

| Vehicle Category | PX4 Model Flag | Mass (kg) | Wingspan | Propulsion Setup | Takeoff Mode | Actuator Mapping |
| :--- | :--- | :--- | :--- | :--- | :--- | :--- |
| **Multicopter (X)** | `iris` | 1.50 kg | — | 4× Brushless Motors (Quad X) | Vertical VTOL | Channels 1..4: Motors |
| **Fixed-Wing UAV** | `plane` | 3.20 kg | 1.80 m | 1× Pusher Propeller + 4 Aileron/Elev/Rud | Runway Ground Roll | Ch 1: Aileron, Ch 2: Elev, Ch 3: Rud, Ch 4: Throttle |
| **QuadPlane VTOL** | `standard_vtol`| 4.50 kg | 2.10 m | 4× VTOL Motors + 1 Pusher Motor + Surfaces | Hybrid VTOL & Cruise | Ch 1..4: VTOL, Ch 5: Pusher, Ch 6..8: Surfaces |

---

## Quickstart Guide

### 1. Prerequisites
- **Operating System**: macOS (Apple Silicon M1/M2/M3/M4 or Intel) or Linux.
- **Unity**: Unity 6 (`6000.3.11f1` or newer) with .NET Standard 2.1 runtime.
- **Container Runtime**: [Colima](https://github.com/abiosoft/colima) with `containerd` runtime, or Apple `container`.
- **.NET SDK**: .NET 8 or 9 SDK (for building the core dynamics assembly).

### 2. Build the Core Dynamics Engine
Verify all 196 automated assertions and build the managed assemblies:

```bash
git clone git@github.com:jyhuh/UnityDrone.git
cd UnityDrone
./scripts/build-and-package.sh
```

### 3. Launch PX4 SITL & Run in Unity

#### Option A: In-Editor 1-Click GUI (Recommended)
1. Open the project in **Unity 6**.
2. Navigate to the `PX4 SITL Process Launcher` component in the Inspector.
3. Select your airframe (`Fixed-Wing (plane)` / `Quad (iris)` / `QuadPlane VTOL (standard_vtol)`).
4. Click **[▶ Launch PX4 SITL Container]**.
5. Press **Play** in Unity.
6. Open **QGroundControl** to monitor telemetry or command autonomous waypoint missions.

#### Option B: Terminal CLI
```bash
# Start the SITL container (e.g. Fixed-Wing plane)
./scripts/px4-sitl.sh up plane

# Check container status
./scripts/px4-sitl.sh status

# Stop containers after session
./scripts/px4-sitl.sh down

# Completely shutdown the background Colima VM
./scripts/px4-sitl.sh vm-stop
```

---

## Technical Documentation Suite

Comprehensive engineering guides, mathematical derivations, and setup manuals are available in the [`docs/`](docs/) directory (compatible with GitHub Wiki and GitHub Pages):

1. [**System Architecture & Multi-Fidelity Solvers**](docs/architecture.md) — Two-stage architecture, CST airfoil modeling, 3D VLM downwash, CCBlade BEMT, and fidelity level trade-offs.
2. [**Ground Contact, Runway Friction & Landing Gear Mechanics**](docs/ground-contact-model.md) — Coulomb rolling resistance, lateral tire grip, takeoff rotation dynamics ($V_{\text{rot}}$), and boundary-layer turbulence attenuation.
3. [**PX4-Autopilot SITL & Container Stack Setup (macOS)**](docs/px4-sitl-setup.md) — Colima containerd optimization, MAVLink lockstep over TCP, dynamic pressure airspeed streaming, and port routing.
4. [**Vehicle Configuration & Parameter Tuning**](docs/vehicle-configuration.md) — Vehicle geometry configuration, aerodynamic polar baking, and actuator channel fallback matrix.
5. [**ROS 2 Humble & Micro-XRCE-DDS Integration**](docs/ros2-integration.md) — DDS topic mapping, offboard trajectory streaming, and Python autonomous orbit controller.

---

## Repository Structure

```
UnityDrone/
├── .github/                       # GitHub Issue & Pull Request Templates
├── Assets/                        # Unity Project Assets & Test Scenes
├── docker/
│   ├── px4-sitl/                  # PX4 v1.15 SITL Containerfile & Startup Scripts
│   └── ros2/                      # ROS 2 Humble & Micro-XRCE-DDS Workspace
├── docs/                          # GitHub Wiki-compatible Technical Documentation
│   ├── README.md                  # Documentation Index & Sitemap
│   ├── architecture.md            # Physics & Mathematical Architecture
│   ├── ground-contact-model.md    # Runway Friction & Rotation Dynamics
│   ├── px4-sitl-setup.md          # macOS Colima SITL Deployment Guide
│   ├── vehicle-configuration.md   # Vehicle Specs & Mixer Calibration
│   └── ros2-integration.md        # ROS 2 Autonomy & uORB Topic Guide
├── dotnet/
│   ├── FlightDynamics6DoF.Core/   # Pure C# .NET Standard 2.1 Core Physics Engine
│   └── FlightDynamics6DoF.Tests/  # 196 Automated Unit & Dynamics Test Assertions
├── Packages/
│   └── com.aerospace.flightdynamics6dof/ # UPM Distribution Package
├── scripts/
│   ├── build-and-package.sh       # Unified Build, Verification & Packaging Script
│   └── px4-sitl.sh                # Container Lifecycle & Colima VM Manager
├── CONTRIBUTING.md                # Conventional Commits & Contribution Guidelines
├── CODE_OF_CONDUCT.md             # Contributor Covenant Code of Conduct
├── LICENSE                        # Apache License 2.0
├── NOTICE                         # Clean-Room Engineering Notice & Attributions
└── README.md                      # Project Overview & Quickstart Guide
```

---

## Community & Contributing

We welcome contributions from aerospace engineers, simulation developers, and roboticists. Please review our guidelines before opening a pull request:
- [**Contributing Guidelines**](CONTRIBUTING.md) — Conventional Commits format, branch naming, and automated test requirements.
- [**Code of Conduct**](CODE_OF_CONDUCT.md) — Contributor Covenant v2.1 standards.
- [**Issue Tracker**](https://github.com/jyhuh/UnityDrone/issues) — File bug reports or submit feature requests using our templates.

---

## License & Intellectual Property

This project is licensed under the **Apache License, Version 2.0**. See the [LICENSE](LICENSE) file for complete terms.

For detailed attribution of underlying academic literature (AirfoilPrep, CCBlade, NeuralFoil, FLOWUnsteady) and clean-room engineering confirmation, see [NOTICE](NOTICE).
