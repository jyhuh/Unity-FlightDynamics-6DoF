# Vehicle Configuration & Actuator Channel Mapping

FlightDynamics 6-DOF supports arbitrary UAV configurations: multirotors, conventional fixed-wing planes, and hybrid VTOL (QuadPlane / Tilt-rotor) platforms.

---

## 1. Supported Reference Airframes

### 1. Standard Quadcopter X (`iris`)
- **Mass**: 1.65 kg | **Arm Length**: 0.22 m
- **Rotors**: 4 $\times$ 9.5-inch 2-blade rotors
- **PX4 Airframe ID**: `none_iris` / `4001_quad_x`
- **Actuator Mapping**:
  - `Motor 1 (Front-Right, CCW)`: Channel 0
  - `Motor 2 (Rear-Left, CCW)`: Channel 1
  - `Motor 3 (Front-Left, CW)`: Channel 2
  - `Motor 4 (Rear-Right, CW)`: Channel 3

### 2. AeroFixedWing Talon-180 Research UAV (`plane`)
- **Mass**: 3.20 kg | **Wing Span**: 1.80 m | **Wing Area**: 0.36 $\text{m}^2$
- **Cruise Airspeed**: 18.0 m/s | **Stall Airspeed**: 11.5 m/s
- **Propulsion**: 1 $\times$ 12-inch forward tractor propeller
- **Surfaces**: Main Wing (Ailerons), Horizontal Stabilizer (Elevator), Vertical Fin (Rudder)
- **PX4 Airframe ID**: `1030_gazebo-classic_plane`

### 3. AeroVTOL QuadPlane Millie-Class Hybrid UAV (`standard_vtol`)
- **Mass**: 5.40 kg | **Wing Span**: 1.80 m
- **Architecture**: 4 $\times$ vertical lift rotors + 1 $\times$ forward cruise pusher propeller
- **PX4 Airframe ID**: `1040_gazebo-classic_standard_vtol`

---

## 2. Actuator Channel Mapping Matrix

PX4 v1.14+ Dynamic Control Allocation assigns actuators according to `PWM_MAIN_FUNC*` parameters:

| Actuator Function | PX4 Parameter | Standard SITL Channel (0-based) | Fixed-Wing Mapping | QuadPlane VTOL Mapping |
| :--- | :--- | :--- | :--- | :--- |
| **VTOL Motor 1 (FR)** | `PWM_MAIN_FUNC1 101` | Channel 0 | *(Unused)* | Lift Rotor 1 (FR) |
| **VTOL Motor 2 (RL)** | `PWM_MAIN_FUNC2 102` | Channel 1 | *(Unused)* | Lift Rotor 2 (RL) |
| **VTOL Motor 3 (FL)** | `PWM_MAIN_FUNC3 103` | Channel 2 | *(Unused)* | Lift Rotor 3 (FL) |
| **VTOL Motor 4 (RR)** | `PWM_MAIN_FUNC4 104` | Channel 3 | *(Unused)* | Lift Rotor 4 (RR) |
| **Cruise Throttle / Motor 1** | `PWM_MAIN_FUNC5 101` | Channel 4 | Tractor Propeller | Cruise Pusher Propeller |
| **Aileron 1** | `PWM_MAIN_FUNC6 201` | Channel 5 | Main Wing Aileron | Main Wing Aileron |
| **Aileron 2** | `PWM_MAIN_FUNC7 202` | Channel 6 | Dual Aileron | Dual Aileron |
| **Elevator** | `PWM_MAIN_FUNC8 203` | Channel 7 | Horizontal Tail Elevator | Horizontal Tail Elevator |
| **Rudder** | `PWM_MAIN_FUNC9 206` | Channel 8 | Vertical Fin Rudder | Vertical Fin Rudder |

---

## 3. Smart Bi-Directional Actuator Fallback

To ensure seamless compatibility between standalone internal testing (channels 0..3) and PX4 SITL (channels 4..8), `MultiFidelitySixDofEngine.SetActuatorControls` features an automatic bi-directional channel bridge:

```csharp
if (_config.Category == VehicleAirframeCategory.FixedWing)
{
    // PX4 SITL output on Channels 4..8 automatically mapped to internal legacy slots
    if (State.ActuatorControlsNormalized[4] > 0.005 && State.ActuatorControlsNormalized[3] <= 0.001)
    {
        State.ActuatorControlsNormalized[3] = State.ActuatorControlsNormalized[4]; // Throttle
        State.ActuatorControlsNormalized[0] = State.ActuatorControlsNormalized[5]; // Aileron
        State.ActuatorControlsNormalized[1] = State.ActuatorControlsNormalized[7]; // Elevator
        State.ActuatorControlsNormalized[2] = State.ActuatorControlsNormalized[8]; // Rudder
    }
    // Reverse fallback: compact input mirrored to PX4 SITL slots
    else if (State.ActuatorControlsNormalized[3] > 0.005 && State.ActuatorControlsNormalized[4] <= 0.001)
    {
        State.ActuatorControlsNormalized[4] = State.ActuatorControlsNormalized[3];
        State.ActuatorControlsNormalized[5] = State.ActuatorControlsNormalized[0];
        State.ActuatorControlsNormalized[7] = State.ActuatorControlsNormalized[1];
        State.ActuatorControlsNormalized[8] = State.ActuatorControlsNormalized[2];
    }
}
```

---

## 4. Creating a Custom Aircraft in C#

You can define a new aircraft specification in C# using fluent builders:

```csharp
var spec = new AircraftConfigSpec
{
    AircraftName = "Custom Tactical UAV",
    Category = VehicleAirframeCategory.FixedWing,
    MassKg = 4.5,
    CenterOfGravityFrd = new Vec3d(-0.08, 0.0, 0.0),
    ReferenceAreaSqM = 0.42,
    ReferenceSpanMeters = 2.10,
    ReferenceChordMeters = 0.20
};

// Add Lifting Surface
spec.Wings.Add(new WingSurfaceSpec
{
    Name = "MainWing",
    Span = 2.10,
    RootChord = 0.24,
    TipChord = 0.16,
    Airfoil = AirfoilSpec.FromNaca4Digit("2412", 5e5),
    ControlType = ControlSurfaceType.Aileron,
    ActuatorChannelIndex = 5
});

// Bake Aerodynamic Polar Database
BakedAeroDatabase db = StabilityAndTrimAnalyzer.AnalyzeAndBake(spec);
```

---

## 5. Workbench Airframe Tuning & Telemetry Logs

- **Step 1 (`AeroDesignPanel`)**: Switch airframes (`iris`, `plane`, `standard_vtol`), run Stage-1 aerodynamic baking, inspect BEMT/2D airfoil polars, or import/export `AircraftConfigSpec` JSON.
- **Step 4 (`Px4TuningPanel`)**: Tune per-airframe `AirframeControlParameterTable` gains (`MC_*`, `FW_*`, `MPC_*`, `TECS_*`) and export `.params` directly to `docker/config/` for PX4 SITL.
- **Step 5 (`TelemetryLogPanel`) & Retention (`FlightLogSettingsWindow`)**: Inspect live telemetry plots and manage `Assets/FlightLogs/` CSV/HTML retention limits via **`Tools > FlightDynamics 6-DOF > Log & Retention Settings...`**.
- **Step 6 (`Ros2Nav2Panel`)**: Configure `Ros2LidarAndObstacleCourse` (2D Raycast LiDAR + test obstacle course), launch or stop **Nav2 + RViz2** (noVNC `http://localhost:6080/vnc.html`), and run the circular Offboard trajectory node.
