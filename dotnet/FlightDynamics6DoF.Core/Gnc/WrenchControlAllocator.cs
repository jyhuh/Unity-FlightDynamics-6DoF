// Licensed under the Apache License, Version 2.0.
using System;
using System.Globalization;
using System.Text;
using FlightDynamics6DoF.Core.Math;
using FlightDynamics6DoF.Core.Specs;

namespace FlightDynamics6DoF.Core.Gnc
{
    /// <summary>
    /// Universal Pseudoinverse Wrench Control Allocator (Actuator Mixer) and PX4 v1.14+ Control Allocation Parameter Exporter.
    /// Automatically maps 6-DOF control wrench commands (Roll, Pitch, Yaw moments + Collective/Forward Thrust)
    /// to arbitrary Multirotor, Fixed-Wing, Tilt-Rotor, and QuadPlane VTOL actuator channels.
    /// </summary>
    public static class WrenchControlAllocator
    {
        public static void BakeAllocatorIntoDatabase(AircraftConfigSpec config, BakedAeroDatabase db)
        {
            int nRotors = config.Rotors.Count;
            db.NumActuators = 16;
            db.FlattenedMixerMatrix = new double[16 * 4]; // [actuatorIdx * 4 + (0:Roll, 1:Pitch, 2:Yaw, 3:VerticalThrust)]

            // Count vertical lift rotors (thrust axis primarily -Z_FRD)
            var liftIndices = new System.Collections.Generic.List<int>();
            for (int i = 0; i < nRotors; i++)
            {
                if (config.Rotors[i].ThrustAxisFrd.Z < -0.35)
                    liftIndices.Add(i);
            }

            if (liftIndices.Count > 0)
            {
                int m = liftIndices.Count;
                // Effectiveness matrix B (4 x m) mapping normalized thrust u_i in [0..1] to [Roll_Nm, Pitch_Nm, Yaw_Nm, Lift_N]
                double[,] B = new double[4, m];
                for (int k = 0; k < m; k++)
                {
                    int rIdx = liftIndices[k];
                    var r = config.Rotors[rIdx];
                    double maxOmega = r.MaxRpm * MathConstants.RpmToRadPerSec;
                    double kt = (rIdx < db.RotorLumpedKt.Length) ? db.RotorLumpedKt[rIdx] : 1.5e-5;
                    double kq = (rIdx < db.RotorLumpedKq.Length) ? db.RotorLumpedKq[rIdx] : 2.5e-7;

                    double maxThrust = System.Math.Max(1.0, kt * maxOmega * maxOmega);
                    double maxTorque = System.Math.Max(0.01, kq * maxOmega * maxOmega);

                    Vec3d arm = r.MountPositionFrd - config.CenterOfGravityFrd;
                    Vec3d fUnit = r.ThrustAxisFrd.Normalized();
                    Vec3d momentPerThrust = Vec3d.Cross(arm, fUnit);
                    Vec3d yawReaction = fUnit * (r.RotationDirection * (maxTorque / maxThrust));
                    Vec3d totalMomentAtMax = (momentPerThrust + yawReaction) * maxThrust;

                    B[0, k] = totalMomentAtMax.X; // Roll moment
                    B[1, k] = totalMomentAtMax.Y; // Pitch moment
                    B[2, k] = totalMomentAtMax.Z; // Yaw moment
                    B[3, k] = -fUnit.Z * maxThrust; // Upward lift force (+Up = -Z_FRD)
                }

                double[,] pinv = LinearAlgebra.ComputeDampedRightPseudoinverse(B, 1e-4);
                for (int k = 0; k < m; k++)
                {
                    int rIdx = liftIndices[k];
                    int ch = config.Rotors[rIdx].ActuatorChannelIndex;
                    if (ch >= 0 && ch < 16)
                    {
                        db.FlattenedMixerMatrix[ch * 4 + 0] = pinv[k, 0];
                        db.FlattenedMixerMatrix[ch * 4 + 1] = pinv[k, 1];
                        db.FlattenedMixerMatrix[ch * 4 + 2] = pinv[k, 2];
                        db.FlattenedMixerMatrix[ch * 4 + 3] = pinv[k, 3];
                    }
                }
            }
        }

        /// <summary>
        /// Allocates desired body moments [Nm], upward collective thrust [N], and forward cruise throttle [0..1]
        /// into the 16 normalized actuator channels.
        /// </summary>
        public static void AllocateActuators(
            AircraftConfigSpec config,
            BakedAeroDatabase db,
            Vec3d desiredMomentFrdNm,
            double desiredUpwardThrustNewtons,
            double desiredForwardThrottleNormalized,
            double airspeedMps,
            Span<double> outActuatorControls16)
        {
            outActuatorControls16.Clear();

            // 1. Allocate vertical lift rotors using pre-baked pseudoinverse mixer
            for (int i = 0; i < config.Rotors.Count; i++)
            {
                var r = config.Rotors[i];
                int ch = r.ActuatorChannelIndex;
                if (ch < 0 || ch >= outActuatorControls16.Length)
                    continue;

                if (r.ThrustAxisFrd.Z < -0.35)
                {
                    double uThrust =
                        db.FlattenedMixerMatrix[ch * 4 + 0] * desiredMomentFrdNm.X +
                        db.FlattenedMixerMatrix[ch * 4 + 1] * desiredMomentFrdNm.Y +
                        db.FlattenedMixerMatrix[ch * 4 + 2] * desiredMomentFrdNm.Z +
                        db.FlattenedMixerMatrix[ch * 4 + 3] * desiredUpwardThrustNewtons;

                    uThrust = System.Math.Clamp(uThrust, 0.0, 1.0);
                    // Since Thrust ~ omega^2, normalized RPM command is sqrt(uThrust)
                    outActuatorControls16[ch] = System.Math.Sqrt(uThrust);
                }
                else if (r.ThrustAxisFrd.X > 0.35)
                {
                    // Forward tractor/pusher propulsion motor
                    outActuatorControls16[ch] = System.Math.Clamp(desiredForwardThrottleNormalized, 0.0, 1.0);
                }
            }

            // 2. Allocate aerodynamic control surfaces (Aileron, Elevator, Rudder)
            if (config.Wings != null && config.Wings.Count > 0)
            {
                double qScale = System.Math.Max(20.0, 0.5 * 1.225 * System.Math.Max(8.0, airspeedMps) * System.Math.Max(8.0, airspeedMps) * config.ReferenceAreaSqM);
                double rollNorm = System.Math.Clamp(desiredMomentFrdNm.X / (qScale * config.ReferenceSpanMeters * 0.08), -1.0, 1.0);
                double pitchNorm = System.Math.Clamp(desiredMomentFrdNm.Y / (qScale * config.ReferenceChordMeters * 0.12), -1.0, 1.0);
                double yawNorm = System.Math.Clamp(desiredMomentFrdNm.Z / (qScale * config.ReferenceSpanMeters * 0.06), -1.0, 1.0);

                for (int i = 0; i < config.Wings.Count; i++)
                {
                    var w = config.Wings[i];
                    int ch = w.ActuatorChannelIndex;
                    if (ch < 0 || ch >= outActuatorControls16.Length)
                        continue;

                    switch (w.ControlType)
                    {
                        case ControlSurfaceType.Aileron:
                            outActuatorControls16[ch] = rollNorm;
                            break;
                        case ControlSurfaceType.Elevator:
                            // Standard aerospace & VlmAndLiftingLineSolver sign convention: +cmd -> +My (Nose-Up)
                            outActuatorControls16[ch] = pitchNorm;
                            break;
                        case ControlSurfaceType.Rudder:
                            // Standard aerospace & VlmAndLiftingLineSolver sign convention: +cmd -> +Mz (Nose-Right)
                            outActuatorControls16[ch] = yawNorm;
                            break;
                    }
                }
            }
        }

        /// <summary>
        /// Generates a PX4 v1.14+ Control Allocation &amp; Airframe startup parameter script (.params / shell format)
        /// directly from the C# <see cref="AircraftConfigSpec"/>, <see cref="BakedAeroDatabase"/>, and <see cref="AirframeControlParameterTable"/>.
        /// </summary>
        public static string GeneratePx4AirframeParameterScript(AircraftConfigSpec config, BakedAeroDatabase db)
        {
            var table = AirframeControlParameterTable.CreateForCategory(config.Category, config, db);
            return table.GeneratePx4ParameterScript(config, db);
        }
    }
}
