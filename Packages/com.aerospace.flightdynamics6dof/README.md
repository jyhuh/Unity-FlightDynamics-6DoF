# FlightDynamics 6-DOF: Drone & UAV Suite (`com.aerospace.flightdynamics6dof`)

Engineering-grade 2-Stage Aerodynamic Analysis & Multi-Fidelity Nonlinear 6-DOF Flight Simulation Suite for Unity (2021.3 LTS & Unity 6).

> **Offline User Manual & API Reference (PDF)**: See `Documentation/FlightDynamics6DoF_Manual.pdf` included in this package for the complete step-by-step setup tutorial, vehicle configuration guide, and C# script reference.

---

## Dual-Mode Execution Architecture

### Mode A: Standalone Built-in C# Autopilot Mode (Default Out-of-the-Box)
- **100% Self-Contained (Windows, macOS, Linux)**: Requires **no external software, containers, or Linux VMs**.
- **Built-in Cascaded Nonlinear GNC (`CascadedFlightController`)**:
  - **Multirotor (`Quad X-450`)**: SO(3) quaternion attitude control, PID velocity/position loops, vertical takeoff, position hold, autonomous circular orbit, Return-to-Launch (RTL), and Auto-Land.
  - **Fixed-Wing (`Talon-180 Research UAV`)**: 3-phase runway takeoff roll with Coulomb friction ($\mu_{\text{roll}} = 0.025$), nose-gear rotation at $V_{\text{rot}}$, and TECS altitude/airspeed + L1 lateral cruise controller.
  - **Hybrid QuadPlane eVTOL (`Millie QuadPlane VTOL`)**: Vertical VTOL hover, forward cruise acceleration, and seamless Multicopter $\leftrightarrow$ Fixed-Wing aerodynamic transition.

### Mode B: Optional External PX4-Autopilot SITL, QGroundControl & ROS 2 Bridge
- **Zero-Dependency MAVLink v2 Networking (`MavlinkNetworkNode`)**:
  - Listens on **`TCP :4560`** for external PX4-Autopilot SITL hardware-in-the-loop lockstep (`HIL_SENSOR`, `HIL_GPS`, `HIL_ACTUATOR_CONTROLS`). When an external PX4 SITL connects, control automatically hands over from the internal C# autopilot to PX4 SITL.
  - Streams telemetry on **`UDP :14550`** for simultaneous **QGroundControl** mission monitoring.
  - Streams 360° 2D Raycast LiDAR scans (`Ros2LidarAndObstacleCourse`) on **`UDP :8890`** for **ROS 2 Humble / Nav2** obstacle avoidance.

---

## Quick Start in Unity Editor

1. **Open the Demo Scene**:
   - Open `Assets/FlightDynamics6DoF/Demo/Scenes/FlightDynamics6DoF_EngineeringDemo.unity` (or click **`Tools > FlightDynamics 6-DOF > Quick-Build 3D Demo Scene Now`**), then press **Play**.
   - In the on-screen **Engineering HUD**, click **`Takeoff`**, **`Start Autonomous Circular Orbit (10m)`**, **`Loiter`**, **`RTL`**, or **`Land`**, and switch live between **`Quad X-450`**, **`Fixed-Wing`**, and **`QuadPlane VTOL`**.
2. **Open the Unified 6-Step Engineering Workbench**:
   - Open **`Tools > FlightDynamics 6-DOF > Unified 5-Step Engineering Workbench`** (or **`Window > Aerospace > Unified 5-Step Engineering Workbench`**):
   - **Tab 1 (`1. Aero/Design`)**: Select airframe presets, extract 3D mesh inertia, run **2D Airfoil Polar (CST + Viterna $\pm 180^\circ$)** and **CCBlade BEMT Rotor** calculators, and bake the **Stage-1 Aerodynamic & Stability Database** ($C_{L_\alpha}, C_{m_\alpha}, C_{n_\beta}, x_{\text{NP}}$, Static Margin % MAC).
   - **Tab 2 (`2. Infra`)**: View Standalone C# Autopilot status or optionally manage external PX4 SITL / ROS 2 containers.
   - **Tab 3 (`3. Environment`)**: Select Stage-2 simulation fidelity (`Level0_UnityRigidbody` through `Level3_6DoFFullAeroWake`), configure wind & Dryden turbulence, and rebuild the 3D scene.
   - **Tab 4 (`4. PX4 Tuning`)**: Tune per-airframe control gains (`AirframeControlParameterTable`), command autonomous maneuvers, and export `.params` files.
   - **Tab 5 (`5. Telemetry`)**: Inspect real-time multi-channel flight plots, RMSE metrics, and export **CSV & Interactive HTML5/SVG Academic Reports** (`Assets/FlightLogs/`).
   - **Tab 6 (`6. ROS2/Nav2`)**: Configure the 360° 2D Raycast LiDAR, generate the 3D obstacle course, and launch circular orbit trajectories.
