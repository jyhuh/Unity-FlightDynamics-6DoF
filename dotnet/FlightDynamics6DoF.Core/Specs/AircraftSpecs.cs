// Licensed under the Apache License, Version 2.0.
using System;
using System.Collections.Generic;
using FlightDynamics6DoF.Core.Math;

namespace FlightDynamics6DoF.Core.Specs
{
    /// <summary>
    /// User-selectable simulation fidelity tiers for Stage 2 (Real-Time Simulation).
    /// </summary>
    public enum SimulationFidelityLevel
    {
        /// <summary>
        /// Level 0: Uses Unity's native Rigidbody (FixedUpdate) with Stage-1 lumped rotor thrust/torque (kT*w^2, kQ*w^2),
        /// bluff body drag, and linearized stability derivatives. Ideal for large drone swarms and lightweight game physics.
        /// </summary>
        Level0_UnityRigidbody = 0,

        /// <summary>
        /// Level 1: High-frequency custom RK4 6-DOF integrator driven by Stage-1 pre-baked aerodynamic lookup tables,
        /// stability/damping derivatives, and BEMT advance-ratio curves. Fast and accurate for SITL and RL training.
        /// </summary>
        Level1_6DoFSurrogate = 1,

        /// <summary>
        /// Level 2: High-frequency custom RK4 6-DOF integrator with real-time per-rotor CCBlade BEMT (including oblique flow
        /// and ground effect), real-time nonlinear lifting-line wing strips (+/-180 deg Viterna + Du-Selig), and slender-body/cross-flow fuselage.
        /// </summary>
        Level2_6DoFBladeAndStrip = 2,

        /// <summary>
        /// Level 3: Maximum engineering fidelity. Level 2 plus real-time 3D VLM horseshoe vortex downwash/sidewash coupling,
        /// contracting rotor slipstream tube interaction on wings/tail (blown-wing transition), and continuous 3-axis Dryden wind turbulence.
        /// </summary>
        Level3_6DoFFullAeroWake = 3
    }

    public enum VehicleAirframeCategory
    {
        Multirotor = 0,
        FixedWing = 1,
        HybridQuadPlaneVtol = 2,
        TiltRotorVtol = 3
    }

    public enum ControlSurfaceType
    {
        None = 0,
        Aileron = 1,
        Elevator = 2,
        Rudder = 3,
        Flap = 4,
        Elevon = 5
    }

    /// <summary>
    /// 2D Airfoil section specification using CST (Class-Shape Transformation) weights and geometric parameters.
    /// </summary>
    [Serializable]
    public class AirfoilSpec
    {
        public string Name = "NACA 2412";
        public double MaxThicknessRatio = 0.12;
        public double MaxCamberRatio = 0.02;
        public double MaxCamberLocation = 0.40;
        public double LeadingEdgeRadius = 0.0158;
        public double TrailingEdgeAngleDeg = 14.0;
        public double DesignReynolds = 5.0e5;
        public double DesignMach = 0.08;

        /// <summary>
        /// 8 Kulfan CST weights for the upper surface (Au_0 .. Au_7).
        /// </summary>
        public double[] CstUpper = new double[] { 0.175, 0.165, 0.180, 0.170, 0.165, 0.160, 0.155, 0.150 };

        /// <summary>
        /// 8 Kulfan CST weights for the lower surface (Al_0 .. Al_7).
        /// </summary>
        public double[] CstLower = new double[] { -0.155, -0.130, -0.115, -0.105, -0.095, -0.085, -0.080, -0.075 };

        public static AirfoilSpec FromNaca4Digit(string code, double designRe = 5.0e5)
        {
            if (string.IsNullOrEmpty(code) || code.Length != 4)
                code = "0012";

            int mDigit = code[0] - '0';
            int pDigit = code[1] - '0';
            int tDigits = int.Parse(code.Substring(2, 2));

            double m = mDigit / 100.0;
            double p = pDigit > 0 ? pDigit / 10.0 : 0.4;
            double t = tDigits / 100.0;

            var spec = new AirfoilSpec
            {
                Name = $"NACA {code}",
                MaxThicknessRatio = t,
                MaxCamberRatio = m,
                MaxCamberLocation = p,
                LeadingEdgeRadius = 1.1019 * t * t,
                TrailingEdgeAngleDeg = 2.0 * System.Math.Atan(1.16925 * t) * FrameConverter.Rad2Deg,
                DesignReynolds = designRe,
                CstUpper = new double[8],
                CstLower = new double[8]
            };

            // Fit 8-term Bernstein CST weights to thickness + camber distribution
            for (int i = 0; i < 8; i++)
            {
                double x = (i + 0.5) / 8.0;
                double baseT = 1.45 * t * (1.0 - 0.12 * x);
                double camberBias = m > 0 ? 2.2 * m * System.Math.Sin(System.Math.PI * System.Math.Pow(x, 0.8)) : 0.0;
                spec.CstUpper[i] = baseT + camberBias;
                spec.CstLower[i] = -baseT + camberBias;
            }
            return spec;
        }
    }

    /// <summary>
    /// 3D Lifting surface specification (Main Wing, Horizontal Tail, Vertical Fin, Canard).
    /// </summary>
    [Serializable]
    public class WingSurfaceSpec
    {
        public string Name = "MainWing";
        public bool IsVerticalFin = false;
        public double Span = 1.80;
        public double RootChord = 0.24;
        public double TipChord = 0.16;
        public Vec3d ApexPositionFrd = new Vec3d(0.05, 0.0, -0.04);
        public double SweepQuarterChordDeg = 2.0;
        public double DihedralDeg = 2.5;
        public double RootIncidenceDeg = 2.0;
        public double TipWashoutDeg = -1.5;
        public int SpanwisePanels = 12;
        public AirfoilSpec Airfoil = AirfoilSpec.FromNaca4Digit("2412", 4.5e5);

        public ControlSurfaceType ControlType = ControlSurfaceType.Aileron;
        public double FlapChordRatio = 0.25;
        public double FlapSpanStartRatio = 0.55;
        public double FlapSpanEndRatio = 0.95;
        public double MaxDeflectionDeg = 22.0;
        public int ActuatorChannelIndex = 4;

        public double PlanformArea => 0.5 * (RootChord + TipChord) * Span;
        public double AspectRatio => PlanformArea > 1e-6 ? (Span * Span) / PlanformArea : 6.0;
        public double TaperRatio => RootChord > 1e-6 ? TipChord / RootChord : 1.0;

        public double MeanAerodynamicChord
        {
            get
            {
                double lambda = TaperRatio;
                return (2.0 / 3.0) * RootChord * (1.0 + lambda + lambda * lambda) / (1.0 + lambda);
            }
        }
    }

    /// <summary>
    /// Fuselage and non-lifting body specification for Slender Body Theory (Multhopp) and Component Build-up parasitic drag.
    /// </summary>
    [Serializable]
    public class FuselageSpec
    {
        public string Name = "Fuselage";
        public double Length = 0.85;
        public double MaxDiameter = 0.14;
        public double WettedArea = 0.34;
        public double InterferenceFactorQ = 1.05;
        public Vec3d CenterPositionFrd = Vec3d.Zero;

        /// <summary>
        /// Axial stations (x/L in [0..1]) and cross-sectional areas S(x) [m^2] for Slender Body destabilizing moment integral.
        /// </summary>
        public double[] AxialStationsNormalized = new double[] { 0.0, 0.1, 0.25, 0.5, 0.75, 0.9, 1.0 };
        public double[] CrossSectionAreas = new double[] { 0.0, 0.008, 0.015, 0.0154, 0.010, 0.004, 0.0005 };

        /// <summary>
        /// Projected frontal/side/planform areas [m^2] in FRD axes for high-alpha / multirotor cross-flow bluff-body drag.
        /// </summary>
        public Vec3d ProjectedAreasFrd = new Vec3d(0.025, 0.045, 0.095);

        /// <summary>
        /// Empirical bluff-body drag coefficients along FRD axes for separated cross-flow.
        /// </summary>
        public Vec3d BluffDragCoefficientsFrd = new Vec3d(0.65, 0.85, 1.15);

        /// <summary>
        /// Additional miscellaneous parasitic drag delta CD0 (exposed wiring, landing skids, camera gimbal).
        /// </summary>
        public double MiscDeltaCd0 = 0.004;
    }

    /// <summary>
    /// Rotor / Propeller specification for CCBlade BEMT and Slipstream wake interaction.
    /// </summary>
    [Serializable]
    public class RotorSpec
    {
        public string Name = "Rotor_1";
        public Vec3d MountPositionFrd = new Vec3d(0.16, 0.16, -0.02);
        public Vec3d ThrustAxisFrd = new Vec3d(0.0, 0.0, -1.0); // Points UP in FRD (negative Down) for vertical lift, or +X for tractor/pusher
        public int RotationDirection = 1; // +1: CW, -1: CCW (viewed from above / along -ThrustAxis)
        public double Radius = 0.127; // 10-inch prop -> R = 0.127 m
        public double HubRadius = 0.018;
        public int NumBlades = 2;

        public double[] RadialStationsNormalized = new double[] { 0.20, 0.35, 0.50, 0.65, 0.80, 0.92, 0.98 };
        public double[] RadialChordsMeters = new double[] { 0.022, 0.027, 0.029, 0.027, 0.023, 0.017, 0.011 };
        public double[] RadialTwistDeg = new double[] { 28.0, 22.5, 17.5, 14.0, 11.2, 9.2, 8.0 };

        public AirfoilSpec BladeAirfoil = AirfoilSpec.FromNaca4Digit("4412", 1.2e5);
        public double MotorTimeConstantSec = 0.025;
        public double RotorMomentOfInertiaKgM2 = 4.5e-5;
        public double MaxRpm = 9500.0;
        public double IdleRpm = 800.0;
        public double MaxMotorPowerWatts = 350.0;
        public double MotorKv = 920.0;
        public int ActuatorChannelIndex = 0;
        public int TiltServoChannelIndex = -1;
        public double MaxTiltDeg = 90.0;
    }

    /// <summary>
    /// Virtual sensor suite parameters (IMU, GPS, Barometer, Magnetometer, Rangefinder).
    /// </summary>
    [Serializable]
    public class SensorSuiteSpec
    {
        public Vec3d ImuMountOffsetFrd = Vec3d.Zero;
        public double AccelNoiseStdMps2 = 0.035;
        public double AccelBiasRandomWalk = 0.002;
        public double GyroNoiseStdRadps = 0.003;
        public double GyroBiasRandomWalk = 0.0001;

        public double BaroNoiseStdHpa = 0.04;
        public Vec3d MagEarthFieldGaussNed = new Vec3d(0.22, 0.02, 0.41);
        public double MagNoiseStdGauss = 0.003;

        public double GpsUpdateRateHz = 10.0;
        public double GpsHorizontalNoiseStdMeters = 0.35;
        public double GpsVerticalNoiseStdMeters = 0.60;
        public double GpsVelocityNoiseStdMps = 0.05;

        public double RangefinderMaxRangeMeters = 40.0;
        public double RangefinderNoiseStdMeters = 0.02;
        public int RandomSeed = 42;
    }

    /// <summary>
    /// Complete data-driven aircraft configuration supporting Multirotors, Fixed-Wing UAVs, and Hybrid VTOL aircraft.
    /// </summary>
    [Serializable]
    public class AircraftConfigSpec
    {
        public string AircraftName = "Engineering Quadcopter X-450";
        public VehicleAirframeCategory Category = VehicleAirframeCategory.Multirotor;
        public SimulationFidelityLevel DefaultFidelity = SimulationFidelityLevel.Level2_6DoFBladeAndStrip;

        public double MassKg = 1.65;
        public Vec3d CenterOfGravityFrd = Vec3d.Zero;
        public double Ixx = 0.0185;
        public double Iyy = 0.0192;
        public double Izz = 0.0335;
        public double Ixz = 0.0004;

        public double ReferenceAreaSqM = 0.12;
        public double ReferenceChordMeters = 0.20;
        public double ReferenceSpanMeters = 0.45;

        public double DesignCruiseSpeedMps = 12.0;
        public double DesignAltitudeMsl = 100.0;

        public bool EnableGroundEffect = true;
        public bool EnableBladeFlappingAndRotorDrag = true;
        public bool EnableDrydenTurbulence = false;
        public double TurbulenceIntensityMps = 0.0;

        public double BatteryCells = 4;
        public double BatteryCapacityMah = 5000.0;
        public double BatteryInternalResistanceOhm = 0.018;
        public string SpecNotes = "";

        /// <summary>
        /// Vertical distance from aircraft CG down to landing gear contact point (wheels or skids) in meters.
        /// When resting on runway, CG is positioned at (GroundElevation - LandingGearHeightMeters) in NED.
        /// Fixed-Wing default ~0.18m (allows 12-inch prop ground clearance), Multirotor ~0.08m.
        /// </summary>
        public double LandingGearHeightMeters = 0.08;

        public FuselageSpec Fuselage = new FuselageSpec();
        public List<WingSurfaceSpec> Wings = new List<WingSurfaceSpec>();
        public List<RotorSpec> Rotors = new List<RotorSpec>();
        public SensorSuiteSpec Sensors = new SensorSuiteSpec();

        public Mat3d GetInertiaTensor() => Mat3d.FromInertiaTensor(Ixx, Iyy, Izz, 0.0, Ixz, 0.0);

        /// <summary>
        /// Factory preset: Standard PX4-numbering Quadcopter X (Front-Right CCW=0, Rear-Left CCW=1, Front-Left CW=2, Rear-Right CW=3).
        /// </summary>
        public static AircraftConfigSpec CreateStandardQuadcopterX()
        {
            var cfg = new AircraftConfigSpec
            {
                AircraftName = "AeroQuad X-450 Engineering Research UAV",
                Category = VehicleAirframeCategory.Multirotor,
                DefaultFidelity = SimulationFidelityLevel.Level2_6DoFBladeAndStrip,
                MassKg = 1.65,
                Ixx = 0.0185,
                Iyy = 0.0192,
                Izz = 0.0335,
                Ixz = 0.0,
                ReferenceAreaSqM = 0.095,
                ReferenceChordMeters = 0.22,
                ReferenceSpanMeters = 0.45,
                DesignCruiseSpeedMps = 10.0,
                Fuselage = new FuselageSpec
                {
                    Name = "QuadCentralHubAndArms",
                    Length = 0.32,
                    MaxDiameter = 0.14,
                    WettedArea = 0.16,
                    ProjectedAreasFrd = new Vec3d(0.022, 0.022, 0.075),
                    BluffDragCoefficientsFrd = new Vec3d(0.85, 0.85, 1.20),
                    MiscDeltaCd0 = 0.01
                }
            };

            double arm = 0.16; // 0.16m X/Y offset (~450mm diagonal)
            // PX4 Standard Quad X motor order:
            // Motor 1 (idx 0): Front-Right (+X, +Y), CCW (-1)
            // Motor 2 (idx 1): Rear-Left   (-X, -Y), CCW (-1)
            // Motor 3 (idx 2): Front-Left  (+X, -Y), CW  (+1)
            // Motor 4 (idx 3): Rear-Right  (-X, +Y), CW  (+1)
            cfg.Rotors.Add(CreateVerticalRotor("Rotor_1_FR", new Vec3d( arm,  arm, -0.02), -1, 0));
            cfg.Rotors.Add(CreateVerticalRotor("Rotor_2_RL", new Vec3d(-arm, -arm, -0.02), -1, 1));
            cfg.Rotors.Add(CreateVerticalRotor("Rotor_3_FL", new Vec3d( arm, -arm, -0.02), +1, 2));
            cfg.Rotors.Add(CreateVerticalRotor("Rotor_4_RR", new Vec3d(-arm,  arm, -0.02), +1, 3));

            return cfg;
        }

        /// <summary>
        /// Factory preset: Fixed-Wing Research UAV (Tractor propeller + Main Wing + Elevator + Rudder).
        /// </summary>
        public static AircraftConfigSpec CreateFixedWingUav()
        {
            var cfg = new AircraftConfigSpec
            {
                AircraftName = "AeroFixedWing Talon-180 Research UAV",
                Category = VehicleAirframeCategory.FixedWing,
                DefaultFidelity = SimulationFidelityLevel.Level3_6DoFFullAeroWake,
                MassKg = 3.20,
                CenterOfGravityFrd = new Vec3d(-0.06, 0.0, 0.0),
                Ixx = 0.185,
                Iyy = 0.240,
                Izz = 0.390,
                Ixz = 0.012,
                ReferenceAreaSqM = 0.36,
                ReferenceChordMeters = 0.20,
                ReferenceSpanMeters = 1.80,
                DesignCruiseSpeedMps = 18.0,
                LandingGearHeightMeters = 0.18,
                Fuselage = new FuselageSpec
                {
                    Name = "StreamlinedPodFuselage",
                    Length = 1.15,
                    MaxDiameter = 0.14,
                    WettedArea = 0.42,
                    ProjectedAreasFrd = new Vec3d(0.018, 0.11, 0.12),
                    BluffDragCoefficientsFrd = new Vec3d(0.25, 0.75, 0.85),
                    MiscDeltaCd0 = 0.003
                }
            };

            // Main Wing (Aerodynamic center slightly behind CG for positive static margin)
            cfg.Wings.Add(new WingSurfaceSpec
            {
                Name = "MainWing",
                IsVerticalFin = false,
                Span = 1.80,
                RootChord = 0.24,
                TipChord = 0.16,
                ApexPositionFrd = new Vec3d(0.03, 0.0, -0.04),
                SweepQuarterChordDeg = 2.0,
                DihedralDeg = 3.0,
                RootIncidenceDeg = 2.5,
                TipWashoutDeg = -1.5,
                SpanwisePanels = 12,
                Airfoil = AirfoilSpec.FromNaca4Digit("2412", 4.5e5),
                ControlType = ControlSurfaceType.Aileron,
                FlapChordRatio = 0.25,
                FlapSpanStartRatio = 0.55,
                FlapSpanEndRatio = 0.95,
                MaxDeflectionDeg = 22.0,
                ActuatorChannelIndex = 5 // PX4 1030_gazebo-classic_plane PWM_MAIN_FUNC6 (controls[5])
            });

            // Horizontal Stabilizer + Elevator
            cfg.Wings.Add(new WingSurfaceSpec
            {
                Name = "HorizontalTail",
                IsVerticalFin = false,
                Span = 0.48,
                RootChord = 0.12,
                TipChord = 0.08,
                ApexPositionFrd = new Vec3d(-0.55, 0.0, -0.05),
                SweepQuarterChordDeg = 4.0,
                DihedralDeg = 0.0,
                RootIncidenceDeg = -1.0,
                TipWashoutDeg = 0.0,
                SpanwisePanels = 6,
                Airfoil = AirfoilSpec.FromNaca4Digit("0009", 2.0e5),
                ControlType = ControlSurfaceType.Elevator,
                FlapChordRatio = 0.35,
                FlapSpanStartRatio = 0.05,
                FlapSpanEndRatio = 0.95,
                MaxDeflectionDeg = 25.0,
                ActuatorChannelIndex = 7 // PX4 1030_gazebo-classic_plane PWM_MAIN_FUNC8 (controls[7])
            });

            // Vertical Fin + Rudder
            cfg.Wings.Add(new WingSurfaceSpec
            {
                Name = "VerticalFin",
                IsVerticalFin = true,
                Span = 0.26,
                RootChord = 0.16,
                TipChord = 0.10,
                ApexPositionFrd = new Vec3d(-0.70, 0.0, -0.15),
                SweepQuarterChordDeg = 12.0,
                DihedralDeg = 0.0,
                RootIncidenceDeg = 0.0,
                TipWashoutDeg = 0.0,
                SpanwisePanels = 6,
                Airfoil = AirfoilSpec.FromNaca4Digit("0009", 2.0e5),
                ControlType = ControlSurfaceType.Rudder,
                FlapChordRatio = 0.35,
                FlapSpanStartRatio = 0.10,
                FlapSpanEndRatio = 0.95,
                MaxDeflectionDeg = 25.0,
                ActuatorChannelIndex = 8 // PX4 1030_gazebo-classic_plane PWM_MAIN_FUNC9 (controls[8])
            });

            // Forward Tractor Propeller (+X thrust axis, PX4 PWM_MAIN_FUNC5: controls[4])
            var pusher = CreateVerticalRotor("TractorPropeller", new Vec3d(0.38, 0.0, 0.0), 1, 4);
            pusher.ThrustAxisFrd = new Vec3d(1.0, 0.0, 0.0);
            pusher.Radius = 0.152; // 12-inch cruise prop
            pusher.RadialTwistDeg = new double[] { 34.0, 28.0, 23.0, 19.0, 15.5, 13.0, 11.5 };
            cfg.Rotors.Add(pusher);

            return cfg;
        }

        /// <summary>
        /// Factory preset: Hybrid QuadPlane VTOL inspired by McGill Robotics "Millie" architecture
        /// (4 vertical lift rotors + 1 forward cruise propeller + Main Wing + H/V Tail).
        /// </summary>
        public static AircraftConfigSpec CreateQuadPlaneVtolMillie()
        {
            var cfg = CreateFixedWingUav();
            cfg.AircraftName = "AeroVTOL QuadPlane Millie-Class Hybrid UAV";
            cfg.Category = VehicleAirframeCategory.HybridQuadPlaneVtol;
            cfg.MassKg = 5.40;
            cfg.Ixx = 0.38;
            cfg.Iyy = 0.45;
            cfg.Izz = 0.72;
            cfg.DesignCruiseSpeedMps = 20.0;

            // Clear and assign PX4 QuadPlane standard channels:
            // Ch 0..3: Quad VTOL Lift Rotors
            // Ch 4: Forward Cruise Propeller
            // Ch 5: Aileron, Ch 6: Elevator, Ch 7: Rudder
            cfg.Rotors.Clear();
            cfg.Rotors.Add(CreateVerticalRotor("VTOL_Rotor_1_FR", new Vec3d( 0.32,  0.42, -0.03), -1, 0, 0.152));
            cfg.Rotors.Add(CreateVerticalRotor("VTOL_Rotor_2_RL", new Vec3d(-0.34, -0.42, -0.03), -1, 1, 0.152));
            cfg.Rotors.Add(CreateVerticalRotor("VTOL_Rotor_3_FL", new Vec3d( 0.32, -0.42, -0.03), +1, 2, 0.152));
            cfg.Rotors.Add(CreateVerticalRotor("VTOL_Rotor_4_RR", new Vec3d(-0.34,  0.42, -0.03), +1, 3, 0.152));

            var cruiseProp = CreateVerticalRotor("Cruise_Pusher_5", new Vec3d(-0.42, 0.0, -0.02), +1, 4, 0.165);
            cruiseProp.ThrustAxisFrd = new Vec3d(1.0, 0.0, 0.0);
            cruiseProp.RadialTwistDeg = new double[] { 36.0, 30.0, 24.5, 20.0, 16.5, 14.0, 12.0 };
            cfg.Rotors.Add(cruiseProp);

            if (cfg.Wings.Count >= 3)
            {
                cfg.Wings[0].ActuatorChannelIndex = 5; // Aileron
                cfg.Wings[1].ActuatorChannelIndex = 6; // Elevator
                cfg.Wings[2].ActuatorChannelIndex = 7; // Rudder
            }

            return cfg;
        }

        private static RotorSpec CreateVerticalRotor(string name, Vec3d posFrd, int dir, int channel, double radius = 0.127)
        {
            return new RotorSpec
            {
                Name = name,
                MountPositionFrd = posFrd,
                ThrustAxisFrd = new Vec3d(0.0, 0.0, -1.0),
                RotationDirection = dir,
                Radius = radius,
                HubRadius = radius * 0.14,
                NumBlades = 2,
                ActuatorChannelIndex = channel,
                MaxRpm = 9800.0
            };
        }
    }

    /// <summary>
    /// Output of Stage 1 (Pre-Simulation Geometry and Aerodynamic Analysis Pipeline).
    /// Feeds Stage 2 (Real-Time Simulation across Level 0 ~ Level 3) and the Engineering Stability Workbench.
    /// </summary>
    [Serializable]
    public class BakedAeroDatabase
    {
        public string AircraftName = "";
        public string AnalysisTimestampUtc = "";

        // 1. Mass & Geometric Summary
        public double TotalMassKg;
        public Vec3d CenterOfGravityFrd;
        public double ReferenceAreaSqM;
        public double MeanAerodynamicChord;
        public double ReferenceSpan;

        // 2. Stage-1 Rotor Lumped & BEMT Surrogate Data (for Level 0 & Level 1)
        public double[] RotorLumpedKt = Array.Empty<double>(); // Thrust N = Kt * omega^2 (rad/s)
        public double[] RotorLumpedKq = Array.Empty<double>(); // Torque N*m = Kq * omega^2 (rad/s)
        public double[] RotorHoverRpm = Array.Empty<double>();
        public double[] AdvanceRatioGrid = Array.Empty<double>();
        public double[] BemtCtCurve = Array.Empty<double>();
        public double[] BemtCqCurve = Array.Empty<double>();
        public double[] BemtEtaCurve = Array.Empty<double>();

        // 3. Stage-1 2D Airfoil Viscous Polar Sample (-180..+180 deg)
        public double[] AirfoilAlphaDegGrid = Array.Empty<double>();
        public double[] AirfoilClGrid = Array.Empty<double>();
        public double[] AirfoilCdGrid = Array.Empty<double>();
        public double[] AirfoilCmGrid = Array.Empty<double>();

        // 4. Stage-1 3D VLM + Fuselage Surrogate Polar (-30..+30 deg)
        public double[] AircraftAlphaDegGrid = Array.Empty<double>();
        public double[] AircraftCLGrid = Array.Empty<double>();
        public double[] AircraftCDGrid = Array.Empty<double>();
        public double[] AircraftCmGrid = Array.Empty<double>();

        // 5. Longitudinal & Lateral-Directional Stability & Control Derivatives (1/rad)
        public double CL0;
        public double CLAlpha;     // dCL / d(alpha)
        public double CLq;         // dCL / d(q * c / 2V)
        public double CLDeltaE;    // dCL / d(delta_e)

        public double CD0;         // Parasitic zero-lift drag (Slender Body + Component Build-up)
        public double InducedDragFactorK; // 1 / (pi * AR * e)
        public double OswaldEfficiency;

        public double Cm0;
        public double CmAlpha;     // Longitudinal pitch stiffness (< 0 for static stability)
        public double Cmq;         // Pitch damping derivative (< 0)
        public double CmDeltaE;    // Elevator control power

        public double CYBeta;      // Side-force derivative
        public double CYp;
        public double CYr;
        public double CYDeltaR;
        public double ClBeta;      // Dihedral effect (< 0 for roll stability)
        public double Clp;         // Roll damping (< 0)
        public double Clr;
        public double ClDeltaA;    // Aileron control power
        public double ClDeltaR;
        public double CnBeta;      // Weathercock yaw stability (> 0 for static directional stability)
        public double Cnp;
        public double Cnr;         // Yaw damping (< 0)
        public double CnDeltaA;
        public double CnDeltaR;    // Rudder control power

        // 6. Stability Metrics & Trim Solution
        public double NeutralPointXFrd;      // x_NP in FRD [m]
        public double StaticMarginFraction;  // SM = (x_CG - x_NP) / MAC = -CmAlpha / CLAlpha (> 0 is stable in FRD where +X is forward)
        public bool IsLongitudinallyStable;
        public bool IsDirectionallyStable;

        public double TrimAirspeedMps;
        public double TrimAlphaDeg;
        public double TrimElevatorDeg;
        public double TrimThrottleNormalized;

        // 7. Precomputed Wrench Control Allocation Matrix (flattened [numActuators * 4]: Roll, Pitch, Yaw, VerticalThrust)
        public int NumActuators;
        public double[] FlattenedMixerMatrix = Array.Empty<double>();
    }
}
