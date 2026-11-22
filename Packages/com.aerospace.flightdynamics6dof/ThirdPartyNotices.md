# Third-Party Notices

`FlightDynamics 6-DOF: Drone & UAV Suite` (`com.aerospace.flightdynamics6dof`) is licensed under the **Apache License, Version 2.0**.

All aerodynamic, flight dynamics, control allocation, and protocol algorithms in `FlightDynamics6DoF.Core.dll` are clean-room C# (.NET Standard 2.1) implementations based strictly on permissive (**Apache 2.0 / MIT**) open-source formulations and public-domain aerospace literature. No GPL/LGPL libraries (such as XFOIL, AVL, QPROP, XFLR5, or OpenFOAM) are included or linked.

## Algorithmic References & Attributions

1. **NREL AirfoilPrep** — Apache License 2.0
   - Viterna high-angle-of-attack ($\pm 180^\circ$) extrapolation and Du-Selig 3D rotational stall delay models.
2. **NeuralFoil & AeroSandbox** (Peter Sharpe, MIT) — MIT License
   - Kulfan CST (Class-Shape Transformation) parameterization, boundary-layer transition correlation, 3D Vortex Lattice Method (VLM), and Slender Body / Component Build-up drag formulations.
3. **MachUpX** (USU AeroLab) — MIT License
   - Nonlinear numerical lifting-line and control surface effectiveness formulations.
4. **CCBlade & FLOWUnsteady** (BYU FLOW Lab) — MIT License
   - Guaranteed-convergence Blade Element Momentum Theory (BEMT) parameterized by inflow angle $\phi$ and contracting rotor slipstream wake interaction models.
5. **mcgill-robotics/drone-SW-Unity-Sim** (McGill Robotics) — MIT License
   - Component-based data-driven vehicle specification and PX4 SITL HIL / Control Allocation export concepts.
6. **MAVLink v2 Wire Protocol** — MIT License
   - Standard MAVLink v2 packet framing and message definitions.
