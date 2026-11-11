// Licensed under the Apache License, Version 2.0.
using System;
using System.Collections.Generic;
using FlightDynamics6DoF.Core.Aerodynamics;
using FlightDynamics6DoF.Core.Dynamics;
using FlightDynamics6DoF.Core.Math;
using FlightDynamics6DoF.Core.Specs;

namespace FlightDynamics6DoF.Core.Gnc
{
    public enum AutopilotFlightMode
    {
        Disarmed = 0,
        StabilizedAttitude = 1,
        AltitudeHold = 2,
        PositionHoldNed = 3,
        OffboardVelocity = 4,
        WaypointMission = 5,
        ReturnToLaunch = 6,
        AutoLand = 7
    }

    [Serializable]
    public struct MissionWaypointNed
    {
        public Vec3d PositionNed;
        public double YawRad;
        public double AcceptanceRadiusMeters;
        public double HoldTimeSec;
    }

    /// <summary>
    /// Built-in Cascaded Nonlinear SO(3) / Quaternion Attitude &amp; PID Position Flight Controller.
    /// Allows standalone autonomous flight and direct QGroundControl mission execution without requiring external Linux PX4 SITL.
    /// </summary>
    public class CascadedFlightController
    {
        public bool IsArmed { get; set; } = false;
        public AutopilotFlightMode CurrentMode { get; set; } = AutopilotFlightMode.PositionHoldNed;

        // Setpoints
        public Vec3d TargetPositionNed { get; set; } = new Vec3d(0.0, 0.0, -10.0);
        public Vec3d TargetVelocityNed { get; set; } = Vec3d.Zero;
        public Quatd TargetAttitudeBodyToNed { get; set; } = Quatd.Identity;
        public double TargetYawRad { get; set; } = 0.0;

        // Tunable Gains (also exposed via MAVLink PARAM_SET / PARAM_VALUE)
        public double KpPosXY { get; set; } = 1.25;
        public double KpPosZ { get; set; } = 1.60;
        public double MaxHorizontalSpeedMps { get; set; } = 8.0;
        public double MaxVerticalSpeedMps { get; set; } = 3.5;

        public double KpVelXY { get; set; } = 3.20;
        public double KiVelXY { get; set; } = 0.45;
        public double KdVelXY { get; set; } = 0.25;

        public double KpVelZ { get; set; } = 5.00;
        public double KiVelZ { get; set; } = 1.20;
        public double KdVelZ { get; set; } = 0.40;

        public double KpAttitudeRollPitch { get; set; } = 6.50;
        public double KpAttitudeYaw { get; set; } = 2.80;

        public double KpRateRollPitch { get; set; } = 0.22;
        public double KiRateRollPitch { get; set; } = 0.08;
        public double KdRateRollPitch { get; set; } = 0.006;

        public double KpRateYaw { get; set; } = 0.15;
        public double KiRateYaw { get; set; } = 0.04;

        private Vec3d _velIntegralNed;
        private Vec3d _prevVelErrorNed;
        private Vec3d _rateIntegralFrd;
        private Vec3d _prevRateErrorFrd;

        private readonly List<MissionWaypointNed> _missionWaypoints = new List<MissionWaypointNed>();
        private int _activeWaypointIndex = 0;

        public IReadOnlyList<MissionWaypointNed> MissionWaypoints => _missionWaypoints;
        public int ActiveWaypointIndex => _activeWaypointIndex;

        public void SetMissionWaypoints(IEnumerable<MissionWaypointNed> waypoints)
        {
            _missionWaypoints.Clear();
            _missionWaypoints.AddRange(waypoints);
            _activeWaypointIndex = 0;
        }

        public void ResetIntegrators()
        {
            _velIntegralNed = Vec3d.Zero;
            _prevVelErrorNed = Vec3d.Zero;
            _rateIntegralFrd = Vec3d.Zero;
            _prevRateErrorFrd = Vec3d.Zero;
        }

        public void Arm(Vec3d currentPosNed, double currentYawRad)
        {
            IsArmed = true;
            ResetIntegrators();
            if (CurrentMode == AutopilotFlightMode.Disarmed)
                CurrentMode = AutopilotFlightMode.PositionHoldNed;
            TargetPositionNed = new Vec3d(currentPosNed.X, currentPosNed.Y, System.Math.Min(currentPosNed.Z, -2.5));
            TargetYawRad = currentYawRad;
        }

        public void Disarm()
        {
            IsArmed = false;
            CurrentMode = AutopilotFlightMode.Disarmed;
            ResetIntegrators();
        }

        public void CommandTakeoff(Vec3d currentPosNed, double currentYawRad, double altitudeAglMeters = 10.0)
        {
            IsArmed = true;
            CurrentMode = AutopilotFlightMode.PositionHoldNed;
            TargetPositionNed = new Vec3d(currentPosNed.X, currentPosNed.Y, -System.Math.Abs(altitudeAglMeters));
            TargetYawRad = currentYawRad;
        }

        public void CommandReturnToLaunch()
        {
            CurrentMode = AutopilotFlightMode.ReturnToLaunch;
            TargetPositionNed = new Vec3d(0.0, 0.0, System.Math.Min(TargetPositionNed.Z, -15.0));
        }

        public void CommandAutoLand(Vec3d currentPosNed)
        {
            CurrentMode = AutopilotFlightMode.AutoLand;
            TargetPositionNed = new Vec3d(currentPosNed.X, currentPosNed.Y, 0.0);
        }

        /// <summary>
        /// Executes one control step and writes the 16 normalized actuator commands into <paramref name="outControls16"/>.
        /// </summary>
        public void ComputeControlStep(
            double dt,
            AircraftConfigSpec config,
            BakedAeroDatabase db,
            SixDofState state,
            Span<double> outControls16)
        {
            outControls16.Clear();
            if (!IsArmed || CurrentMode == AutopilotFlightMode.Disarmed || dt <= 1e-6)
                return;

            // 1. High-level Navigation / Guidance Loop
            UpdateGuidanceSetpoints(state);

            double mass = System.Math.Max(0.1, config.MassKg);
            double gravity = IsaAtmosphereAndTurbulence.GravityMps2;

            // Fixed-Wing / Hybrid VTOL Forward Cruise Transition handling
            if (config.Category == VehicleAirframeCategory.FixedWing)
            {
                ComputeFixedWingControl(dt, config, db, state, outControls16);
                return;
            }

            // 2. Outer Position Loop -> Desired NED Velocity
            Vec3d velDesNed = TargetVelocityNed;
            if (CurrentMode == AutopilotFlightMode.PositionHoldNed ||
                CurrentMode == AutopilotFlightMode.WaypointMission ||
                CurrentMode == AutopilotFlightMode.ReturnToLaunch)
            {
                Vec3d posErr = TargetPositionNed - state.PositionNed;
                Vec3d vHoriz = new Vec3d(posErr.X * KpPosXY, posErr.Y * KpPosXY, 0.0);
                vHoriz = Vec3d.ClampMagnitude(vHoriz, MaxHorizontalSpeedMps);
                double vz = System.Math.Max(-MaxVerticalSpeedMps, System.Math.Min(MaxVerticalSpeedMps, posErr.Z * KpPosZ));
                velDesNed = new Vec3d(vHoriz.X, vHoriz.Y, vz);
            }
            else if (CurrentMode == AutopilotFlightMode.AutoLand)
            {
                Vec3d posErr = TargetPositionNed - state.PositionNed;
                velDesNed = new Vec3d(
                    System.Math.Max(-2.0, System.Math.Min(2.0, posErr.X * KpPosXY)),
                    System.Math.Max(-2.0, System.Math.Min(2.0, posErr.Y * KpPosXY)),
                    0.65); // Descend at 0.65 m/s (+Down)
                if (state.IsGrounded && state.AltitudeAglMeters < 0.05)
                {
                    Disarm();
                    return;
                }
            }

            // 3. Velocity PID Loop -> Desired NED Acceleration & Total Thrust Vector
            Quatd qDes = TargetAttitudeBodyToNed;
            double collectiveThrustN = mass * gravity;

            if (CurrentMode != AutopilotFlightMode.StabilizedAttitude)
            {
                Vec3d velErr = velDesNed - state.VelocityNed;
                _velIntegralNed += velErr * dt;
                _velIntegralNed = Vec3d.ClampMagnitude(_velIntegralNed, 4.0);
                Vec3d velDeriv = (velErr - _prevVelErrorNed) / dt;
                _prevVelErrorNed = velErr;

                double ax = KpVelXY * velErr.X + KiVelXY * _velIntegralNed.X + KdVelXY * velDeriv.X;
                double ay = KpVelXY * velErr.Y + KiVelXY * _velIntegralNed.Y + KdVelXY * velDeriv.Y;
                double az = KpVelZ * velErr.Z + KiVelZ * _velIntegralNed.Z + KdVelZ * velDeriv.Z;

                // Clamp tilt acceleration to max 35 deg tilt
                double maxHorizAccel = gravity * System.Math.Tan(35.0 * FrameConverter.Deg2Rad);
                Vec3d aHoriz = Vec3d.ClampMagnitude(new Vec3d(ax, ay, 0.0), maxHorizAccel);
                double aUp = System.Math.Max(0.2 * gravity, System.Math.Min(2.2 * gravity, gravity - az));

                double desiredRoll = System.Math.Atan2(
                    -aHoriz.X * System.Math.Sin(TargetYawRad) + aHoriz.Y * System.Math.Cos(TargetYawRad),
                    gravity);
                double desiredPitch = System.Math.Atan2(
                    -aHoriz.X * System.Math.Cos(TargetYawRad) - aHoriz.Y * System.Math.Sin(TargetYawRad),
                    gravity);

                qDes = Quatd.FromEulerZyx(desiredRoll, desiredPitch, TargetYawRad);
                Vec3d bodyUpNed = state.AttitudeBodyToNed.RotateBodyToNed(new Vec3d(0.0, 0.0, -1.0));
                double cosTilt = System.Math.Max(0.5, -bodyUpNed.Z);
                collectiveThrustN = (mass * aUp) / cosTilt;
            }

            // 4. SO(3) / Quaternion Attitude Loop -> Desired Body Angular Rates (p, q, r)
            Vec3d attErrBody = Quatd.AttitudeErrorBody(state.AttitudeBodyToNed, qDes);
            Vec3d rateDesBody = new Vec3d(
                KpAttitudeRollPitch * attErrBody.X,
                KpAttitudeRollPitch * attErrBody.Y,
                KpAttitudeYaw * attErrBody.Z);

            // 5. Inner Body Rate PID Loop -> Desired Moments [Mx, My, Mz]
            Vec3d rateErr = rateDesBody - state.AngularVelocityBodyFrd;
            _rateIntegralFrd += rateErr * dt;
            _rateIntegralFrd = Vec3d.ClampMagnitude(_rateIntegralFrd, 1.5);
            Vec3d rateDeriv = (rateErr - _prevRateErrorFrd) / dt;
            _prevRateErrorFrd = rateErr;

            // Scale gains by inertia tensor so default gains adapt to micro-drones and heavy VTOLs alike
            double scaleX = System.Math.Max(1.0, config.Ixx / 0.02);
            double scaleY = System.Math.Max(1.0, config.Iyy / 0.02);
            double scaleZ = System.Math.Max(1.0, config.Izz / 0.035);

            Vec3d gyroFeedforward = Vec3d.Cross(state.AngularVelocityBodyFrd, config.GetInertiaTensor().Multiply(state.AngularVelocityBodyFrd));
            Vec3d desiredMomentNm = new Vec3d(
                scaleX * (KpRateRollPitch * rateErr.X + KiRateRollPitch * _rateIntegralFrd.X + KdRateRollPitch * rateDeriv.X),
                scaleY * (KpRateRollPitch * rateErr.Y + KiRateRollPitch * _rateIntegralFrd.Y + KdRateRollPitch * rateDeriv.Y),
                scaleZ * (KpRateYaw * rateErr.Z + KiRateYaw * _rateIntegralFrd.Z)) + gyroFeedforward;

            double forwardThrottle = (config.Category == VehicleAirframeCategory.HybridQuadPlaneVtol && velDesNed.Magnitude > 5.0)
                ? System.Math.Min(0.85, velDesNed.Magnitude / System.Math.Max(10.0, config.DesignCruiseSpeedMps))
                : 0.0;

            WrenchControlAllocator.AllocateActuators(
                config, db, desiredMomentNm, collectiveThrustN, forwardThrottle, state.AirspeedMps, outControls16);
        }

        private void ComputeFixedWingControl(
            double dt,
            AircraftConfigSpec config,
            BakedAeroDatabase db,
            SixDofState state,
            Span<double> outControls16)
        {
            Vec3d posErr = TargetPositionNed - state.PositionNed;
            double desiredCourse = System.Math.Atan2(posErr.Y, posErr.X);
            Vec3d euler = state.AttitudeBodyToNed.ToEulerZyx();

            double headingErr = desiredCourse - euler.Z;
            while (headingErr > System.Math.PI) headingErr -= 2.0 * System.Math.PI;
            while (headingErr < -System.Math.PI) headingErr += 2.0 * System.Math.PI;

            double desiredRoll = System.Math.Max(-0.6, System.Math.Min(0.6, 1.2 * headingErr));
            double altErrUp = -posErr.Z;
            double desiredPitch = (db.TrimAlphaDeg * FrameConverter.Deg2Rad) +
                                  System.Math.Max(-0.35, System.Math.Min(0.35, 0.05 * altErrUp));

            Quatd qDes = Quatd.FromEulerZyx(desiredRoll, desiredPitch, euler.Z);
            Vec3d attErr = Quatd.AttitudeErrorBody(state.AttitudeBodyToNed, qDes);
            Vec3d rateDes = new Vec3d(4.0 * attErr.X, 4.0 * attErr.Y, 1.5 * attErr.Z);
            Vec3d rateErr = rateDes - state.AngularVelocityBodyFrd;

            Vec3d momentCmd = new Vec3d(
                2.5 * rateErr.X,
                3.5 * rateErr.Y,
                1.5 * rateErr.Z);

            double speedErr = config.DesignCruiseSpeedMps - state.AirspeedMps;
            double throttle = System.Math.Max(0.15, System.Math.Min(1.0, db.TrimThrottleNormalized + 0.08 * speedErr));

            WrenchControlAllocator.AllocateActuators(
                config, db, momentCmd, 0.0, throttle, state.AirspeedMps, outControls16);
        }

        private void UpdateGuidanceSetpoints(SixDofState state)
        {
            if (CurrentMode == AutopilotFlightMode.WaypointMission && _missionWaypoints.Count > 0)
            {
                if (_activeWaypointIndex >= _missionWaypoints.Count)
                    _activeWaypointIndex = 0;

                var wp = _missionWaypoints[_activeWaypointIndex];
                TargetPositionNed = wp.PositionNed;
                if (!double.IsNaN(wp.YawRad))
                    TargetYawRad = wp.YawRad;

                double dist = (state.PositionNed - wp.PositionNed).Magnitude;
                double radius = System.Math.Max(1.0, wp.AcceptanceRadiusMeters);
                if (dist <= radius && _activeWaypointIndex < _missionWaypoints.Count - 1)
                {
                    _activeWaypointIndex++;
                }
            }
            else if (CurrentMode == AutopilotFlightMode.ReturnToLaunch)
            {
                Vec3d horizErr = new Vec3d(state.PositionNed.X, state.PositionNed.Y, 0.0);
                if (horizErr.Magnitude < 1.5)
                {
                    CommandAutoLand(Vec3d.Zero);
                }
            }
        }
    }
}
