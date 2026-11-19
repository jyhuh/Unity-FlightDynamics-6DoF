# FlightDynamics 6-DOF Documentation

Welcome to the technical documentation and developer guides for **FlightDynamics 6-DOF**.

This documentation covers mathematical formulations, multi-fidelity solver architectures, containerized SITL workflows on macOS, and vehicle configuration tutorials for aerospace simulation in Unity 6.

---

## Documentation Index

### 1. Core Physics & Architecture
- [**System Architecture & Multi-Fidelity Solvers**](architecture.md)
  - Two-Stage Architecture: Offline Aerodynamic Baking vs. Real-Time RK4 Simulation
  - Module 1: 2D Airfoil Viscous Polar Solver (CST + Viterna $\pm 180^\circ$ + Du-Selig)
  - Module 2: 3D Vortex Lattice Method (VLM Horseshoe Wake Induction)
  - Module 3: Fuselage Slender Body Theory & Cross-Flow Bluff-Body Drag
  - Module 4: CCBlade Blade Element Momentum Theory (BEMT) & Contracting Wake Slipstream
  - Module 5: Automated Stability Derivatives, Neutral Point & Static Margin
  - Simulation Fidelity Levels 0..3 Comparison
- [**Ground Contact, Runway Friction & Landing Gear Mechanics**](ground-contact-model.md)
  - Paved Runway Coulomb Rolling Resistance ($\mu_{\text{roll}} \approx 0.025$)
  - Lateral Tire Cornering Grip & Heading Drift Stabilization
  - Parking Brake / Ground Idle Hold (Preventing Wind Slide)
  - Nose Gear Rotation Pitch Freedom & Tailstrike Angle Constraints
  - Atmospheric Ground Boundary Layer Gust Attenuation

---

### 2. Autopilot Integration & Simulation
- [**PX4-Autopilot SITL & Container Stack Setup (macOS)**](px4-sitl-setup.md)
  - Zero-Docker Architecture via Colima (`containerd` + `vz` + `virtiofs`) & Apple `container`
  - MAVLink v2 Lockstep Hardware-in-the-Loop Protocol (`TCP :4560`)
  - Dynamic Pressure ($\frac{1}{2}\rho V^2$) Airspeed Streaming for EKF2 & TECS
  - QGroundControl Ground Control Station Telemetry (`UDP :14550`)
  - Troubleshooting Container Lifecycle (`up`, `down`, `vm-stop`)
- [**ROS 2 Humble & Micro-XRCE-DDS Integration**](ros2-integration.md)
  - Micro-XRCE-DDS Agent Architecture (`UDP :8888`)
  - Live uORB Topic Introspection (`/fmu/in/*` and `/fmu/out/*`)
  - Autonomous Trajectory Offboard Node (`unity_px4_offboard_node.py`)
  - Nav2 + RViz2 (noVNC) Autonomous Obstacle Avoidance & Unity 2D Raycast LiDAR (`UDP :8890`)

---

### 3. Vehicle Configuration & Calibration
- [**Vehicle Specification & Parameter Tuning Guide**](vehicle-configuration.md)
  - Standard Quadcopter X (`iris`)
  - Talon-180 Research UAV Fixed-Wing (`plane`)
  - Millie-Class Hybrid QuadPlane VTOL (`standard_vtol`)
  - Actuator Channel Mapping Matrix & Smart Bi-Directional Fallback
  - Unified 6-Step Engineering Workbench (`FlightDynamicsEngineeringWizard`, `SitlInfraController`, & Step Panels) and Log Retention Settings (`FlightLogSettingsWindow`)

### 4. Project Task & Roadmap
- [**Task & Roadmap Backlog**](roadmap.md)
  - Unimplemented feature backlog (Airframe Designer Popup, Top-bar Infra Quick Controls, Altitude Wind/Gust Layers)
  - Pending verification & End-to-End testing checklist (ROS 2 + Nav2 + RViz2 noVNC, Fixed-Wing/VTOL SITL)

---

## Quick Navigation for Developers
- Looking to quickly start simulation? See [Quickstart Guide in README](../README.md#quickstart-guide).
- Need to contribute code? Read [CONTRIBUTING.md](../CONTRIBUTING.md).
- Questions or bugs? Open an issue using our [GitHub Issue Templates](../.github/ISSUE_TEMPLATE/).
