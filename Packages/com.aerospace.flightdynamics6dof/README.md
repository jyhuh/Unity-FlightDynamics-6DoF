# FlightDynamics 6-DOF (`com.aerospace.flightdynamics6dof`)

Engineering-grade 2-Stage Aerodynamic Analysis & Multi-Fidelity Nonlinear 6-DOF Flight Simulation Suite for Unity.

## Quick Start in Unity Editor

1. Open **`Tools > FlightDynamics 6-DOF > Engineering Workbench & Setup Wizard`**.
2. In **Tab 1 (Stage-1 Aero & Stability Analyzer)**:
   - Choose a reference airframe (`Quadcopter X-450`, `Fixed-Wing Talon-180`, or `QuadPlane VTOL Millie-Class`) or extract geometry from your 3D mesh.
   - Use the interactive **2D Airfoil Polar Calculator (CST + NeuralFoil + Viterna $\pm 180^\circ$)** and **CCBlade BEMT Rotor Calculator**.
   - Click **"Execute Stage 1 Full Aircraft Analysis & Bake Database"** to compute stability derivatives ($C_{L_\alpha}, C_{m_\alpha}, C_{n_\beta}, C_{l_p}, C_{m_q}, C_{n_r}$), **Neutral Point ($x_{\text{NP}}$)**, **Static Margin (% MAC)**, and **Trim state**.
3. In **Tab 2 (Stage-2 Multi-Fidelity 6-DOF)**:
   - Select your runtime accuracy tier from `Level0_UnityRigidbody` up to `Level3_6DoFFullAeroWake`.
4. In **Tab 3 (GNC, MAVLink & PX4 Exporter)**:
   - Export ready-to-use PX4 v1.14+ Control Allocation scripts (`.params`) or JSON vehicle specs.
5. In **Tab 4 (One-Click Scene Setup Wizard)**:
   - Click **"Build Complete 6-DOF Flight Simulation Environment in Current Scene"** and press **Play**!
