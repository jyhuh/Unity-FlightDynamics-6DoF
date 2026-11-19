# FlightDynamics 6-DOF (`com.aerospace.flightdynamics6dof`)

Engineering-grade 2-Stage Aerodynamic Analysis & Multi-Fidelity Nonlinear 6-DOF Flight Simulation Suite for Unity.

## Quick Start in Unity Editor

1. Open **`Tools > FlightDynamics 6-DOF > Unified 5-Step Engineering Workbench`** (6-tab UI).
2. **Tab 1 (`1. Aero/Design`)**:
   - Choose a reference airframe (`Multirotor (Quad X-450 / iris)`, `Fixed-Wing (Talon-180 / plane)`, or `VTOL (QuadPlane / standard_vtol)`), extract geometry from 3D meshes, or import/export `AircraftConfigSpec` JSON.
   - Use the **2D Airfoil Polar Calculator** and **CCBlade BEMT Rotor Calculator**, and run **Stage-1 Aero Analysis & Bake DB** to compute stability derivatives ($C_{L_\alpha}, C_{m_\alpha}, C_{n_\beta}, C_{l_p}, C_{m_q}, C_{n_r}$), **Neutral Point ($x_{\text{NP}}$)**, **Static Margin (% MAC)**, and **Trim state**.
3. **Tab 2 (`2. Infra`)**:
   - Click **`▶ Start All Infra`** to launch the Colima/Apple container runtime, `unity-ros2-dds`, and `unity-px4-sitl` while monitoring live status badges in the header.
4. **Tab 3 (`3. Environment`)**:
   - Select your Stage-2 simulation fidelity (`Level0_UnityRigidbody` through `Level3_6DoFFullAeroWake`), configure wind/turbulence and scene setup, and click **`▶ Start Simulation`** to enter Play Mode.
5. **Tab 4 (`4. PX4 Tuning`)**:
   - Tune per-airframe `AirframeControlParameterTable` gains, export `.params` files for PX4 v1.14+, and send `pxh>` commands directly to PX4 SITL.
6. **Tab 5 (`5. Telemetry`) & Tab 6 (`6. ROS2/Nav2`)**:
   - Inspect live telemetry plots and CSV/HTML logs (`Assets/FlightLogs/`), or configure `Ros2LidarAndObstacleCourse` and launch **ROS 2 Humble + Nav2 + RViz2** (`http://localhost:6080/vnc.html`).
