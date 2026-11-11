// Licensed under the Apache License, Version 2.0.
using System;
using System.Collections.Generic;
using FlightDynamics6DoF.Core.Aerodynamics;
using FlightDynamics6DoF.Core.Math;
using FlightDynamics6DoF.Core.Specs;

namespace FlightDynamics6DoF.Core.Dynamics
{
    public enum SimulationClockMode
    {
        RealTimeAsync = 0,
        SitlLockstep = 1,
        FastForwardBatch = 2
    }

    [Serializable]
    public class SixDofState
    {
        public double SimTimeSec;
        public Vec3d PositionNed = Vec3d.Zero;
        public Vec3d VelocityNed = Vec3d.Zero;
        public Vec3d VelocityBodyFrd = Vec3d.Zero;
        public Quatd AttitudeBodyToNed = Quatd.Identity;
        public Vec3d AngularVelocityBodyFrd = Vec3d.Zero;

        public Vec3d LinearAccelerationBodyFrd = Vec3d.Zero;
        public Vec3d SpecificForceBodyFrd = new Vec3d(0.0, 0.0, -9.80665);
        public Vec3d AngularAccelerationBodyFrd = Vec3d.Zero;

        public Vec3d TotalAeroPropForceFrd = Vec3d.Zero;
        public Vec3d TotalAeroPropMomentCgFrd = Vec3d.Zero;

        public double[] RotorRpm = Array.Empty<double>();
        public double[] RotorOmegaRadps = Array.Empty<double>();
        public double[] RotorThrustNewtons = Array.Empty<double>();
        public double[] ActuatorControlsNormalized = new double[16];

        public double AirspeedMps;
        public double AlphaRad;
        public double BetaRad;
        public double AltitudeAglMeters;
        public bool IsGrounded = true;

        public double BatteryVoltageVolts = 16.8;
        public double BatteryCurrentAmps = 1.0;
        public double BatteryRemainingFraction = 1.0;
    }

    /// <summary>
    /// Stage 2: Real-Time Multi-Fidelity Nonlinear 6-DOF Flight Dynamics Engine.
    /// Supports seamless runtime switching between Level 0 (Unity Rigidbody Force Provider),
    /// Level 1 (RK4 6-DOF + Pre-baked Surrogate Derivatives),
    /// Level 2 (RK4 6-DOF + Real-time CCBlade BEMT &amp; Nonlinear Lifting-Line Strips),
    /// and Level 3 (RK4 6-DOF + Real-time 3D VLM Vortex Wake &amp; Rotor Slipstream Coupling + Dryden Turbulence).
    /// </summary>
    public class MultiFidelitySixDofEngine
    {
        private AircraftConfigSpec _config;
        private BakedAeroDatabase _bakedDb;
        private List<WingPanelElement> _wingPanels;
        private Mat3d _inertia;
        private Mat3d _inertiaInv;
        private readonly IsaAtmosphereAndTurbulence _atmosphere;
        private RotorBemtSolution[] _latestRotorSolutions;

        public AircraftConfigSpec Config => _config;
        public BakedAeroDatabase BakedDatabase => _bakedDb;
        public SixDofState State { get; private set; }
        public SimulationFidelityLevel FidelityLevel { get; set; }
        public SimulationClockMode ClockMode { get; set; } = SimulationClockMode.RealTimeAsync;

        public Vec3d MeanWindNed { get; set; } = Vec3d.Zero;
        public double GroundElevationNedDown { get; set; } = 0.0;
        public double OriginLatitudeDeg { get; set; } = 37.5665;
        public double OriginLongitudeDeg { get; set; } = 126.9780;
        public double OriginAltitudeMsl { get; set; } = 50.0;

        public MultiFidelitySixDofEngine(AircraftConfigSpec config, BakedAeroDatabase? preBakedDatabase = null)
        {
            _config = config ?? throw new ArgumentNullException(nameof(config));
            _bakedDb = preBakedDatabase ?? StabilityAndTrimAnalyzer.AnalyzeAndBake(_config);
            _wingPanels = VlmAndLiftingLineSolver.BuildPanels(_config.Wings);
            _inertia = _config.GetInertiaTensor();
            _inertiaInv = _inertia.Inverse();
            _atmosphere = new IsaAtmosphereAndTurbulence(_config.Sensors.RandomSeed);
            FidelityLevel = _config.DefaultFidelity;

            State = new SixDofState();
            _latestRotorSolutions = new RotorBemtSolution[_config.Rotors.Count];
            ResetState(Vec3d.Zero, Quatd.Identity);
        }

        public void Reconfigure(AircraftConfigSpec config, BakedAeroDatabase? preBakedDatabase = null)
        {
            _config = config;
            _bakedDb = preBakedDatabase ?? StabilityAndTrimAnalyzer.AnalyzeAndBake(_config);
            _wingPanels = VlmAndLiftingLineSolver.BuildPanels(_config.Wings);
            _inertia = _config.GetInertiaTensor();
            _inertiaInv = _inertia.Inverse();
            _latestRotorSolutions = new RotorBemtSolution[_config.Rotors.Count];
            FidelityLevel = _config.DefaultFidelity;
            ResetState(State.PositionNed, State.AttitudeBodyToNed);
        }

        public void ResetState(Vec3d positionNed, Quatd attitudeBodyToNed, Vec3d velocityBodyFrd = default)
        {
            int nRotors = _config.Rotors.Count;
            State.SimTimeSec = 0.0;
            State.PositionNed = positionNed;
            State.AttitudeBodyToNed = attitudeBodyToNed.Normalized();
            State.VelocityBodyFrd = velocityBodyFrd;
            State.VelocityNed = State.AttitudeBodyToNed.RotateBodyToNed(velocityBodyFrd);
            State.AngularVelocityBodyFrd = Vec3d.Zero;
            State.LinearAccelerationBodyFrd = Vec3d.Zero;
            State.AngularAccelerationBodyFrd = Vec3d.Zero;
            State.SpecificForceBodyFrd = State.AttitudeBodyToNed.RotateNedToBody(new Vec3d(0.0, 0.0, -IsaAtmosphereAndTurbulence.GravityMps2));

            State.RotorRpm = new double[nRotors];
            State.RotorOmegaRadps = new double[nRotors];
            State.RotorThrustNewtons = new double[nRotors];
            State.BatteryVoltageVolts = _config.BatteryCells * 4.2;
            State.BatteryCurrentAmps = 0.5;
            State.BatteryRemainingFraction = 1.0;
            State.AltitudeAglMeters = System.Math.Max(0.0, GroundElevationNedDown - positionNed.Z);
            State.IsGrounded = State.AltitudeAglMeters <= 0.02;
        }

        /// <summary>
        /// Sets normalized actuator commands (16 channels: [0..1] for motors/throttles, [-1..+1] for control surfaces).
        /// </summary>
        public void SetActuatorControls(ReadOnlySpan<double> controls)
        {
            int len = System.Math.Min(controls.Length, State.ActuatorControlsNormalized.Length);
            for (int i = 0; i < len; i++)
                State.ActuatorControlsNormalized[i] = controls[i];

            if (_config.Category == VehicleAirframeCategory.FixedWing)
            {
                // Bi-directional fallback for Fixed-Wing between PX4 standard (ch4: throttle, ch5: aileron, ch7: elevator, ch8: rudder)
                // and legacy compact channel assignments (ch0: aileron, ch1: elevator, ch2: rudder, ch3: throttle)
                if (State.ActuatorControlsNormalized[4] > 0.005 && State.ActuatorControlsNormalized[3] <= 0.001)
                {
                    State.ActuatorControlsNormalized[3] = State.ActuatorControlsNormalized[4];
                    State.ActuatorControlsNormalized[0] = State.ActuatorControlsNormalized[5];
                    State.ActuatorControlsNormalized[1] = State.ActuatorControlsNormalized[7];
                    State.ActuatorControlsNormalized[2] = State.ActuatorControlsNormalized[8];
                }
                else if (State.ActuatorControlsNormalized[3] > 0.005 && State.ActuatorControlsNormalized[4] <= 0.001)
                {
                    State.ActuatorControlsNormalized[4] = State.ActuatorControlsNormalized[3];
                    State.ActuatorControlsNormalized[5] = State.ActuatorControlsNormalized[0];
                    State.ActuatorControlsNormalized[7] = State.ActuatorControlsNormalized[1];
                    State.ActuatorControlsNormalized[8] = State.ActuatorControlsNormalized[2];
                }
            }
        }

        /// <summary>
        /// Advances the simulation by <paramref name="dt"/> seconds using the active <see cref="FidelityLevel"/>.
        /// For Level 1~3, executes high-frequency RK4 sub-stepping (default maxSubStep = 0.0025s = 400Hz).
        /// For Level 0, computes lumped forces &amp; torques for Unity's Rigidbody.
        /// </summary>
        public void StepSimulation(double dt, double maxRk4SubStepSec = 0.0025)
        {
            if (dt <= 0.0) return;

            if (FidelityLevel == SimulationFidelityLevel.Level0_UnityRigidbody)
            {
                StepMotorAndBatteryDynamics(dt, Vec3d.Zero);
                double altMsl = OriginAltitudeMsl - State.PositionNed.Z;
                var atm = IsaAtmosphereAndTurbulence.Evaluate(altMsl);
                ComputeTotalAeroAndPropulsionWrench(
                    State.PositionNed, State.VelocityBodyFrd, State.AttitudeBodyToNed, State.AngularVelocityBodyFrd,
                    atm, Vec3d.Zero, SimulationFidelityLevel.Level0_UnityRigidbody,
                    out Vec3d fAeroProp, out Vec3d mAeroProp, out _);

                State.TotalAeroPropForceFrd = fAeroProp;
                State.TotalAeroPropMomentCgFrd = mAeroProp;
                UpdateAirDataMetrics(State.VelocityBodyFrd, Vec3d.Zero);
                State.SimTimeSec += dt;
                return;
            }

            int subSteps = System.Math.Max(1, (int)System.Math.Ceiling(dt / System.Math.Max(0.0005, maxRk4SubStepSec)));
            double h = dt / subSteps;

            double currentAltMsl = OriginAltitudeMsl - State.PositionNed.Z;
            var currentAtm = IsaAtmosphereAndTurbulence.Evaluate(currentAltMsl);
            Vec3d gustBody = (_config.EnableDrydenTurbulence && FidelityLevel >= SimulationFidelityLevel.Level2_6DoFBladeAndStrip)
                ? _atmosphere.StepDrydenTurbulenceBody(dt, State.AirspeedMps, State.AltitudeAglMeters, _config.TurbulenceIntensityMps)
                : Vec3d.Zero;

            for (int s = 0; s < subSteps; s++)
            {
                StepMotorAndBatteryDynamics(h, out Vec3d motorInertialReactionTorqueFrd);
                StepRk4Single(h, currentAtm, gustBody, motorInertialReactionTorqueFrd);
                State.SimTimeSec += h;
            }
        }

        /// <summary>
        /// Synchronizes internal 6-DOF kinematic state from Unity's Rigidbody when running in <see cref="SimulationFidelityLevel.Level0_UnityRigidbody"/>,
        /// and returns the resulting Force and Torque vectors directly in Unity's Left-Handed local coordinates.
        /// </summary>
        public void EvaluateLevel0UnityRigidbodyWrench(
            double dt,
            Vec3d unityWorldPos,
            Quatd unityWorldRot,
            Vec3d unityLocalVel,
            Vec3d unityLocalAngularVel,
            out Vec3d outUnityLocalForce,
            out Vec3d outUnityLocalTorque)
        {
            State.PositionNed = FrameConverter.UnityToNedPosition(unityWorldPos);
            State.AttitudeBodyToNed = FrameConverter.UnityToNedRotation(unityWorldRot);
            State.VelocityBodyFrd = FrameConverter.UnityToFrdVector(unityLocalVel);
            State.VelocityNed = State.AttitudeBodyToNed.RotateBodyToNed(State.VelocityBodyFrd);
            State.AngularVelocityBodyFrd = new Vec3d(-unityLocalAngularVel.Z, -unityLocalAngularVel.X, unityLocalAngularVel.Y);
            State.AltitudeAglMeters = System.Math.Max(0.0, GroundElevationNedDown - State.PositionNed.Z);
            State.IsGrounded = State.AltitudeAglMeters <= 0.02;

            StepSimulation(dt);

            outUnityLocalForce = FrameConverter.FrdToUnityVector(State.TotalAeroPropForceFrd);
            outUnityLocalTorque = FrameConverter.FrdToUnityTorque(State.TotalAeroPropMomentCgFrd);
        }

        private void StepRk4Single(double h, AtmosphereState atm, Vec3d gustBodyFrd, Vec3d motorAccelReactionTorqueFrd)
        {
            Vec3d p0 = State.PositionNed;
            Vec3d v0 = State.VelocityBodyFrd;
            Quatd q0 = State.AttitudeBodyToNed;
            Vec3d w0 = State.AngularVelocityBodyFrd;

            // k1
            EvaluateStateDerivatives(p0, v0, q0, w0, atm, gustBodyFrd, motorAccelReactionTorqueFrd,
                out Vec3d dp1, out Vec3d dv1, out Quatd dq1, out Vec3d dw1,
                out Vec3d fAero1, out Vec3d mAero1, out Vec3d specForce1);

            // Running full 3D VLM matrix inside intermediate RK4 stages k2..k4 is unnecessary overhead because vortex wake
            // evolves on convective time scales >> 1ms; freeze aero wrench across sub-stages when in Level 3 for 4x speed!
            SimulationFidelityLevel subStageFidelity = FidelityLevel == SimulationFidelityLevel.Level3_6DoFFullAeroWake
                ? SimulationFidelityLevel.Level2_6DoFBladeAndStrip
                : FidelityLevel;

            var savedFidelity = FidelityLevel;
            FidelityLevel = subStageFidelity;

            // k2
            Vec3d p2 = p0 + 0.5 * h * dp1;
            Vec3d v2 = v0 + 0.5 * h * dv1;
            Quatd q2 = Quatd.AddScaled(q0, dq1, 0.5 * h);
            Vec3d w2 = w0 + 0.5 * h * dw1;
            EvaluateStateDerivatives(p2, v2, q2, w2, atm, gustBodyFrd, motorAccelReactionTorqueFrd,
                out Vec3d dp2, out Vec3d dv2, out Quatd dq2, out Vec3d dw2, out _, out _, out _);

            // k3
            Vec3d p3 = p0 + 0.5 * h * dp2;
            Vec3d v3 = v0 + 0.5 * h * dv2;
            Quatd q3 = Quatd.AddScaled(q0, dq2, 0.5 * h);
            Vec3d w3 = w0 + 0.5 * h * dw2;
            EvaluateStateDerivatives(p3, v3, q3, w3, atm, gustBodyFrd, motorAccelReactionTorqueFrd,
                out Vec3d dp3, out Vec3d dv3, out Quatd dq3, out Vec3d dw3, out _, out _, out _);

            // k4
            Vec3d p4 = p0 + h * dp3;
            Vec3d v4 = v0 + h * dv3;
            Quatd q4 = Quatd.AddScaled(q0, dq3, h);
            Vec3d w4 = w0 + h * dw3;
            EvaluateStateDerivatives(p4, v4, q4, w4, atm, gustBodyFrd, motorAccelReactionTorqueFrd,
                out Vec3d dp4, out Vec3d dv4, out Quatd dq4, out Vec3d dw4, out _, out _, out _);

            FidelityLevel = savedFidelity;

            double inv6 = h / 6.0;
            State.PositionNed = p0 + inv6 * (dp1 + 2.0 * dp2 + 2.0 * dp3 + dp4);
            State.VelocityBodyFrd = v0 + inv6 * (dv1 + 2.0 * dv2 + 2.0 * dv3 + dv4);

            Quatd dqTotal = new Quatd(
                (dq1.W + 2.0 * dq2.W + 2.0 * dq3.W + dq4.W) / 6.0,
                (dq1.X + 2.0 * dq2.X + 2.0 * dq3.X + dq4.X) / 6.0,
                (dq1.Y + 2.0 * dq2.Y + 2.0 * dq3.Y + dq4.Y) / 6.0,
                (dq1.Z + 2.0 * dq2.Z + 2.0 * dq3.Z + dq4.Z) / 6.0);
            State.AttitudeBodyToNed = Quatd.AddScaled(q0, dqTotal, h);
            State.AngularVelocityBodyFrd = w0 + inv6 * (dw1 + 2.0 * dw2 + 2.0 * dw3 + dw4);

            // Ground plane contact geometric constraints and normal reaction force
            Vec3d vNed = State.AttitudeBodyToNed.RotateBodyToNed(State.VelocityBodyFrd);
            if (State.PositionNed.Z > GroundElevationNedDown)
            {
                State.PositionNed = new Vec3d(State.PositionNed.X, State.PositionNed.Y, GroundElevationNedDown);
                if (vNed.Z > 0.0)
                {
                    vNed.Z = 0.0;
                    State.VelocityBodyFrd = State.AttitudeBodyToNed.RotateNedToBody(vNed);
                }
            }

            Vec3d fAeroNed = State.AttitudeBodyToNed.RotateBodyToNed(fAero1);
            double weightN = _config.MassKg * IsaAtmosphereAndTurbulence.GravityMps2;
            double netDownForceN = fAeroNed.Z + weightN;

            // When sitting on the ground with net downward load, ground normal reaction supports weight
            if (State.PositionNed.Z >= GroundElevationNedDown - 0.005 && netDownForceN > 0.0)
            {
                Vec3d groundNormalNed = new Vec3d(0.0, 0.0, -netDownForceN);
                double invMass = 1.0 / System.Math.Max(0.05, _config.MassKg);
                specForce1 = (fAero1 + State.AttitudeBodyToNed.RotateNedToBody(groundNormalNed)) * invMass;

                bool isFixedWingOrHasWheels = _config.Category == VehicleAirframeCategory.FixedWing ||
                                              _config.Category == VehicleAirframeCategory.HybridQuadPlaneVtol;

                if (isFixedWingOrHasWheels)
                {
                    // Parking Brake / Static Ground Hold when throttle is near idle (< 5%)
                    bool isThrottleIdle = State.ActuatorControlsNormalized[4] < 0.05 && State.ActuatorControlsNormalized[3] < 0.05;
                    if (isThrottleIdle)
                    {
                        // Hold aircraft firmly on runway when resting / disarmed so wind gusts do not slide or blow it away
                        vNed.X *= System.Math.Max(0.0, 1.0 - 25.0 * h);
                        vNed.Y *= System.Math.Max(0.0, 1.0 - 25.0 * h);
                        State.AngularVelocityBodyFrd *= System.Math.Max(0.0, 1.0 - 25.0 * h);
                        State.VelocityBodyFrd = State.AttitudeBodyToNed.RotateNedToBody(vNed);
                    }
                    else
                    {
                        // Landing gear pitch attitude constraints during takeoff ground roll:
                        // Prevent nose gear penetration below runway (euler.Y > 0.015 rad is nose-down into ground in aerospace FRD)
                        // while allowing takeoff rotation (euler.Y < 0 is nose-up)
                        Vec3d euler = State.AttitudeBodyToNed.ToEulerZyx();
                        if (euler.Y > 0.015 && State.AngularVelocityBodyFrd.Y > 0.0)
                        {
                            State.AngularVelocityBodyFrd = new Vec3d(State.AngularVelocityBodyFrd.X, 0.0, State.AngularVelocityBodyFrd.Z);
                        }
                    }
                }
                else
                {
                    // Multicopter resting on landing skids: hold firmly on the landing pad
                    vNed.X *= System.Math.Max(0.0, 1.0 - 30.0 * h);
                    vNed.Y *= System.Math.Max(0.0, 1.0 - 30.0 * h);
                    State.AngularVelocityBodyFrd *= System.Math.Max(0.0, 1.0 - 30.0 * h);
                    State.VelocityBodyFrd = State.AttitudeBodyToNed.RotateNedToBody(vNed);
                }
            }

            State.VelocityNed = State.AttitudeBodyToNed.RotateBodyToNed(State.VelocityBodyFrd);
            State.LinearAccelerationBodyFrd = dv1;
            State.AngularAccelerationBodyFrd = dw1;
            State.SpecificForceBodyFrd = specForce1;
            State.TotalAeroPropForceFrd = fAero1;
            State.TotalAeroPropMomentCgFrd = mAero1;
            State.AltitudeAglMeters = System.Math.Max(0.0, GroundElevationNedDown - State.PositionNed.Z);
            State.IsGrounded = State.AltitudeAglMeters <= 0.02;

            UpdateAirDataMetrics(State.VelocityBodyFrd, gustBodyFrd);
        }

        private void EvaluateStateDerivatives(
            Vec3d posNed,
            Vec3d velBodyFrd,
            Quatd qBodyToNed,
            Vec3d omegaBodyFrd,
            AtmosphereState atm,
            Vec3d gustBodyFrd,
            Vec3d motorAccelReactionTorqueFrd,
            out Vec3d dPosNed,
            out Vec3d dVelBodyFrd,
            out Quatd dQuat,
            out Vec3d dOmegaBodyFrd,
            out Vec3d fAeroPropFrd,
            out Vec3d mAeroPropFrd,
            out Vec3d specificForceBodyFrd)
        {
            ComputeTotalAeroAndPropulsionWrench(
                posNed, velBodyFrd, qBodyToNed, omegaBodyFrd, atm, gustBodyFrd, FidelityLevel,
                out fAeroPropFrd, out mAeroPropFrd, out Vec3d rotorSpinMomentumFrd);

            // Add Ground Spring-Damper Contact Force only if touching/penetrating runway (depth > 0)
            Vec3d fContactFrd = Vec3d.Zero;
            Vec3d mContactFrd = Vec3d.Zero;
            double penetration = posNed.Z - GroundElevationNedDown;
            if (penetration > 0.0)
            {
                double depth = penetration;
                Vec3d vNed = qBodyToNed.RotateBodyToNed(velBodyFrd);
                double kSpring = _config.MassKg * 120.0;
                double cDamper = _config.MassKg * 22.0;
                double normalForceUp = System.Math.Max(0.0, kSpring * depth + cDamper * System.Math.Max(0.0, vNed.Z));

                bool isRollingWheeled = _config.Category == VehicleAirframeCategory.FixedWing ||
                                        _config.Category == VehicleAirframeCategory.HybridQuadPlaneVtol;

                double fxFrictionNed;
                double fyFrictionNed;
                if (isRollingWheeled)
                {
                    // Wheeled aircraft runway rolling resistance: mu_roll ~ 0.025 on paved runway
                    double muRoll = 0.025;
                    double fRoll = muRoll * normalForceUp;
                    fxFrictionNed = -fRoll * System.Math.Tanh(3.0 * vNed.X);

                    // Lateral tire steering grip with symmetric Coulomb limit (mu_lat ~ 0.7)
                    double maxFy = normalForceUp * 0.7;
                    fyFrictionNed = -System.Math.Max(-maxFy, System.Math.Min(maxFy, cDamper * 0.5 * vNed.Y));
                }
                else
                {
                    // Multicopter on landing pad skids
                    fxFrictionNed = -cDamper * 0.25 * vNed.X;
                    fyFrictionNed = -cDamper * 0.25 * vNed.Y;
                }

                Vec3d fContactNed = new Vec3d(fxFrictionNed, fyFrictionNed, -normalForceUp);
                fContactFrd = qBodyToNed.RotateNedToBody(fContactNed);

                // Tricycle landing gear ground contact restores roll/yaw alignment while rolling
                if (isRollingWheeled)
                {
                    mContactFrd = new Vec3d(
                        -omegaBodyFrd.X * (_config.MassKg * 2.5),
                        -omegaBodyFrd.Y * (_config.MassKg * 0.3),
                        -omegaBodyFrd.Z * (_config.MassKg * 2.0));
                }
                else
                {
                    mContactFrd = -omegaBodyFrd * (_config.MassKg * 0.4);
                }
            }

            Vec3d totalNonGravForceFrd = fAeroPropFrd + fContactFrd;
            Vec3d totalMomentCgFrd = mAeroPropFrd + mContactFrd + motorAccelReactionTorqueFrd;

            double invMass = 1.0 / System.Math.Max(0.05, _config.MassKg);
            specificForceBodyFrd = totalNonGravForceFrd * invMass;

            Vec3d gravityNed = new Vec3d(0.0, 0.0, IsaAtmosphereAndTurbulence.GravityMps2);
            Vec3d gravityBodyFrd = qBodyToNed.RotateNedToBody(gravityNed);

            // Newton-Euler translational & rotational dynamics in rotating FRD frame
            dPosNed = qBodyToNed.RotateBodyToNed(velBodyFrd);
            dVelBodyFrd = specificForceBodyFrd + gravityBodyFrd - Vec3d.Cross(omegaBodyFrd, velBodyFrd);
            dQuat = qBodyToNed.Derivative(omegaBodyFrd);

            // Gyroscopic term: omega x (I * omega + h_rotors)
            Vec3d totalAngMom = _inertia.Multiply(omegaBodyFrd) + rotorSpinMomentumFrd;
            Vec3d gyroTorque = Vec3d.Cross(omegaBodyFrd, totalAngMom);
            dOmegaBodyFrd = _inertiaInv.Multiply(totalMomentCgFrd - gyroTorque);
        }

        private void ComputeTotalAeroAndPropulsionWrench(
            Vec3d posNed,
            Vec3d velBodyFrd,
            Quatd qBodyToNed,
            Vec3d omegaBodyFrd,
            AtmosphereState atm,
            Vec3d gustBodyFrd,
            SimulationFidelityLevel fidelity,
            out Vec3d totalForceFrd,
            out Vec3d totalMomentCgFrd,
            out Vec3d rotorSpinAngularMomentumFrd)
        {
            Vec3d windBodyFrd = qBodyToNed.RotateNedToBody(MeanWindNed) + gustBodyFrd;
            Vec3d vAirRelFrd = velBodyFrd - windBodyFrd;
            double altAgl = System.Math.Max(0.0, GroundElevationNedDown - posNed.Z);

            totalForceFrd = Vec3d.Zero;
            totalMomentCgFrd = Vec3d.Zero;
            rotorSpinAngularMomentumFrd = Vec3d.Zero;

            int nRotors = _config.Rotors.Count;

            // 1. Propulsion / Rotor Evaluation
            for (int i = 0; i < nRotors; i++)
            {
                var r = _config.Rotors[i];
                double omega = State.RotorOmegaRadps[i];
                double rpm = State.RotorRpm[i];

                // Rotor spin angular momentum h_i = I_rot * omega_i * dir * thrustAxis
                rotorSpinAngularMomentumFrd += r.ThrustAxisFrd * (r.RotationDirection * r.RotorMomentOfInertiaKgM2 * omega);

                if (fidelity == SimulationFidelityLevel.Level0_UnityRigidbody ||
                    fidelity == SimulationFidelityLevel.Level1_6DoFSurrogate)
                {
                    // Fast Stage-1 Lumped / Surrogate Rotor Force & Torque
                    double kt = (i < _bakedDb.RotorLumpedKt.Length) ? _bakedDb.RotorLumpedKt[i] : 1.5e-5;
                    double kq = (i < _bakedDb.RotorLumpedKq.Length) ? _bakedDb.RotorLumpedKq[i] : 2.5e-7;

                    double thrust = kt * omega * omega;
                    double torque = kq * omega * omega;

                    // Ground effect multiplier
                    if (_config.EnableGroundEffect && r.ThrustAxisFrd.Z < -0.5 && altAgl > 0.05)
                    {
                        double zOverR = System.Math.Max(0.65, altAgl / r.Radius);
                        if (zOverR < 4.0)
                        {
                            double ratio = 1.0 / (4.0 * zOverR);
                            thrust /= System.Math.Max(0.72, 1.0 - ratio * ratio);
                        }
                    }

                    State.RotorThrustNewtons[i] = thrust;
                    Vec3d fRotor = r.ThrustAxisFrd.Normalized() * thrust;

                    // Rotor H-drag in surrogate mode
                    if (_config.EnableBladeFlappingAndRotorDrag)
                    {
                        Vec3d vHub = vAirRelFrd + Vec3d.Cross(omegaBodyFrd, r.MountPositionFrd - _config.CenterOfGravityFrd);
                        Vec3d vInPlane = vHub - Vec3d.Dot(vHub, r.ThrustAxisFrd) * r.ThrustAxisFrd;
                        fRotor += -0.018 * thrust * vInPlane;
                    }

                    Vec3d mRotor = r.ThrustAxisFrd.Normalized() * (r.RotationDirection * torque);
                    Vec3d arm = r.MountPositionFrd - _config.CenterOfGravityFrd;

                    totalForceFrd += fRotor;
                    totalMomentCgFrd += Vec3d.Cross(arm, fRotor) + mRotor;
                }
                else
                {
                    // Level 2 & Level 3: Real-time CCBlade BEMT per rotor
                    double tiltRad = 0.0;
                    if (r.TiltServoChannelIndex >= 0 && r.TiltServoChannelIndex < State.ActuatorControlsNormalized.Length)
                        tiltRad = State.ActuatorControlsNormalized[r.TiltServoChannelIndex] * r.MaxTiltDeg * FrameConverter.Deg2Rad;

                    var sol = CcBladeBemtSolver.EvaluateRotor(
                        r, vAirRelFrd, omegaBodyFrd, rpm, atm, altAgl,
                        _config.EnableGroundEffect, _config.EnableBladeFlappingAndRotorDrag, tiltRad);

                    _latestRotorSolutions[i] = sol;
                    State.RotorThrustNewtons[i] = sol.ThrustNewtons;

                    Vec3d arm = r.MountPositionFrd - _config.CenterOfGravityFrd;
                    totalForceFrd += sol.ForceFrd;
                    totalMomentCgFrd += Vec3d.Cross(arm, sol.ForceFrd) + sol.MomentAboutMountFrd;
                }
            }

            // 2. Airframe Aerodynamics (Wings + Fuselage)
            if (fidelity == SimulationFidelityLevel.Level0_UnityRigidbody ||
                fidelity == SimulationFidelityLevel.Level1_6DoFSurrogate)
            {
                EvaluateSurrogateAirframeAero(vAirRelFrd, omegaBodyFrd, atm, out Vec3d fAero, out Vec3d mAero);
                totalForceFrd += fAero;
                totalMomentCgFrd += mAero;
            }
            else if (fidelity == SimulationFidelityLevel.Level2_6DoFBladeAndStrip)
            {
                // Level 2: Nonlinear Lifting-Line Strips (+/-180 deg Viterna) + Slender Body / Cross-Flow
                if (_wingPanels.Count > 0)
                {
                    var wingRes = VlmAndLiftingLineSolver.Evaluate(
                        _config.Wings, _wingPanels, vAirRelFrd, omegaBodyFrd, _config.CenterOfGravityFrd,
                        atm, State.ActuatorControlsNormalized, null, null, enable3DVlmWakeInduction: false);
                    totalForceFrd += wingRes.TotalForceFrd;
                    totalMomentCgFrd += wingRes.TotalMomentAboutCgFrd;
                }

                var fusRes = FuselageAndParasiticSolver.Evaluate(
                    _config.Fuselage, vAirRelFrd, _config.CenterOfGravityFrd,
                    _config.ReferenceAreaSqM, _config.ReferenceChordMeters, _config.ReferenceSpanMeters, atm);
                totalForceFrd += fusRes.ForceFrd;
                totalMomentCgFrd += fusRes.MomentAboutCgFrd;
            }
            else
            {
                // Level 3: Full 3D VLM Horseshoe Vortex Downwash/Sidewash + Rotor Slipstream Tube Coupling + Slender Body
                if (_wingPanels.Count > 0)
                {
                    var wingRes = VlmAndLiftingLineSolver.Evaluate(
                        _config.Wings, _wingPanels, vAirRelFrd, omegaBodyFrd, _config.CenterOfGravityFrd,
                        atm, State.ActuatorControlsNormalized, _config.Rotors, _latestRotorSolutions, enable3DVlmWakeInduction: true);
                    totalForceFrd += wingRes.TotalForceFrd;
                    totalMomentCgFrd += wingRes.TotalMomentAboutCgFrd;
                }

                var fusRes = FuselageAndParasiticSolver.Evaluate(
                    _config.Fuselage, vAirRelFrd, _config.CenterOfGravityFrd,
                    _config.ReferenceAreaSqM, _config.ReferenceChordMeters, _config.ReferenceSpanMeters, atm);
                totalForceFrd += fusRes.ForceFrd;
                totalMomentCgFrd += fusRes.MomentAboutCgFrd;
            }
        }

        private void EvaluateSurrogateAirframeAero(
            Vec3d vAirRelFrd,
            Vec3d omegaBodyFrd,
            AtmosphereState atm,
            out Vec3d fAeroFrd,
            out Vec3d mAeroFrd)
        {
            double vMag = vAirRelFrd.Magnitude;
            if (vMag < 0.05)
            {
                fAeroFrd = Vec3d.Zero;
                mAeroFrd = Vec3d.Zero;
                return;
            }

            // Bluff-body drag always active at all airspeeds
            var fusRes = FuselageAndParasiticSolver.Evaluate(
                _config.Fuselage, vAirRelFrd, _config.CenterOfGravityFrd,
                _config.ReferenceAreaSqM, _config.ReferenceChordMeters, _config.ReferenceSpanMeters, atm);

            if (_config.Wings == null || _config.Wings.Count == 0)
            {
                fAeroFrd = fusRes.ForceFrd;
                mAeroFrd = fusRes.MomentAboutCgFrd - 0.008 * omegaBodyFrd;
                return;
            }

            double alpha = System.Math.Atan2(vAirRelFrd.Z, System.Math.Max(0.2, System.Math.Abs(vAirRelFrd.X)));
            double beta = System.Math.Asin(System.Math.Max(-1.0, System.Math.Min(1.0, vAirRelFrd.Y / vMag)));

            double qDyn = 0.5 * atm.DensityKgPerM3 * vMag * vMag;
            double sRef = _bakedDb.ReferenceAreaSqM;
            double cRef = _bakedDb.MeanAerodynamicChord;
            double bRef = _bakedDb.ReferenceSpan;
            double qS = qDyn * sRef;

            double pHat = (omegaBodyFrd.X * bRef) / (2.0 * System.Math.Max(2.0, vMag));
            double qHat = (omegaBodyFrd.Y * cRef) / (2.0 * System.Math.Max(2.0, vMag));
            double rHat = (omegaBodyFrd.Z * bRef) / (2.0 * System.Math.Max(2.0, vMag));

            // Extract control deflections
            double da = 0.0, de = 0.0, dr = 0.0;
            for (int i = 0; i < _config.Wings.Count; i++)
            {
                var w = _config.Wings[i];
                int ch = w.ActuatorChannelIndex;
                if (ch < 0 || ch >= State.ActuatorControlsNormalized.Length) continue;
                double rad = State.ActuatorControlsNormalized[ch] * w.MaxDeflectionDeg * FrameConverter.Deg2Rad;
                if (w.ControlType == ControlSurfaceType.Aileron) da = rad;
                else if (w.ControlType == ControlSurfaceType.Elevator) de = rad;
                else if (w.ControlType == ControlSurfaceType.Rudder) dr = rad;
            }

            double cl = _bakedDb.CL0 + _bakedDb.CLAlpha * alpha + _bakedDb.CLq * qHat + _bakedDb.CLDeltaE * de;
            double cd = _bakedDb.CD0 + _bakedDb.InducedDragFactorK * cl * cl;
            double cy = _bakedDb.CYBeta * beta + _bakedDb.CYp * pHat + _bakedDb.CYr * rHat + _bakedDb.CYDeltaR * dr;

            double cRoll = _bakedDb.ClBeta * beta + _bakedDb.Clp * pHat + _bakedDb.Clr * rHat + _bakedDb.ClDeltaA * da + _bakedDb.ClDeltaR * dr;
            double cPitch = _bakedDb.Cm0 + _bakedDb.CmAlpha * alpha + _bakedDb.Cmq * qHat + _bakedDb.CmDeltaE * de;
            double cYaw = _bakedDb.CnBeta * beta + _bakedDb.Cnp * pHat + _bakedDb.Cnr * rHat + _bakedDb.CnDeltaA * da + _bakedDb.CnDeltaR * dr;

            double lift = qS * cl;
            double drag = qS * cd;
            double side = qS * cy;

            double cosA = System.Math.Cos(alpha);
            double sinA = System.Math.Sin(alpha);

            fAeroFrd = new Vec3d(
                -drag * cosA + lift * sinA,
                side,
                -drag * sinA - lift * cosA);

            mAeroFrd = new Vec3d(
                qS * bRef * cRoll,
                qS * cRef * cPitch,
                qS * bRef * cYaw);
        }

        private void StepMotorAndBatteryDynamics(double dt, out Vec3d motorInertialReactionTorqueFrd)
        {
            motorInertialReactionTorqueFrd = Vec3d.Zero;
            int nRotors = _config.Rotors.Count;
            double totalPowerWatts = 15.0; // Avionics baseline draw

            for (int i = 0; i < nRotors; i++)
            {
                var r = _config.Rotors[i];
                int ch = r.ActuatorChannelIndex;
                double cmd = (ch >= 0 && ch < State.ActuatorControlsNormalized.Length)
                    ? System.Math.Max(0.0, System.Math.Min(1.0, State.ActuatorControlsNormalized[ch]))
                    : 0.0;

                double targetRpm = cmd > 0.01
                    ? r.IdleRpm + cmd * (r.MaxRpm - r.IdleRpm)
                    : 0.0;

                double tau = System.Math.Max(0.002, r.MotorTimeConstantSec);
                double alphaLag = 1.0 - System.Math.Exp(-dt / tau);
                double prevOmega = State.RotorOmegaRadps[i];

                double newRpm = State.RotorRpm[i] + alphaLag * (targetRpm - State.RotorRpm[i]);
                double newOmega = newRpm * (2.0 * System.Math.PI / 60.0);
                double omegaDot = dt > 1e-6 ? (newOmega - prevOmega) / dt : 0.0;

                State.RotorRpm[i] = newRpm;
                State.RotorOmegaRadps[i] = newOmega;

                // Reactive inertial torque from rotor acceleration: -I_rot * omega_dot
                motorInertialReactionTorqueFrd += r.ThrustAxisFrd.Normalized() * (r.RotationDirection * r.RotorMomentOfInertiaKgM2 * omegaDot);

                double kq = (i < _bakedDb.RotorLumpedKq.Length) ? _bakedDb.RotorLumpedKq[i] : 2.5e-7;
                totalPowerWatts += kq * newOmega * newOmega * newOmega / 0.82;
            }

            double openCircuitVoltage = _config.BatteryCells * (3.5 + 0.7 * State.BatteryRemainingFraction);
            State.BatteryCurrentAmps = totalPowerWatts / System.Math.Max(9.0, openCircuitVoltage);
            State.BatteryVoltageVolts = System.Math.Max(
                _config.BatteryCells * 3.1,
                openCircuitVoltage - State.BatteryCurrentAmps * _config.BatteryInternalResistanceOhm);

            double capacityCoulombs = System.Math.Max(100.0, _config.BatteryCapacityMah * 3.6);
            State.BatteryRemainingFraction = System.Math.Max(0.0, State.BatteryRemainingFraction - (State.BatteryCurrentAmps * dt) / capacityCoulombs);
        }

        private void StepMotorAndBatteryDynamics(double dt, Vec3d _) => StepMotorAndBatteryDynamics(dt, out _);

        private void UpdateAirDataMetrics(Vec3d velBodyFrd, Vec3d gustBodyFrd)
        {
            Vec3d vAir = velBodyFrd - (State.AttitudeBodyToNed.RotateNedToBody(MeanWindNed) + gustBodyFrd);
            double vMag = vAir.Magnitude;
            State.AirspeedMps = vMag;
            if (vMag > 0.1)
            {
                State.AlphaRad = System.Math.Atan2(vAir.Z, System.Math.Max(0.05, System.Math.Abs(vAir.X)));
                State.BetaRad = System.Math.Asin(System.Math.Max(-1.0, System.Math.Min(1.0, vAir.Y / vMag)));
            }
            else
            {
                State.AlphaRad = 0.0;
                State.BetaRad = 0.0;
            }
        }
    }
}
