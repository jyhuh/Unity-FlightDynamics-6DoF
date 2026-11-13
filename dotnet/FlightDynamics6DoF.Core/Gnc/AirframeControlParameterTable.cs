// Licensed under the Apache License, Version 2.0.
using System;
using System.Collections.Generic;
using System.Globalization;
using System.Text;
using FlightDynamics6DoF.Core.Specs;

namespace FlightDynamics6DoF.Core.Gnc
{
    [Serializable]
    public class ControlParameterEntry
    {
        public string Name = "";
        public float Value;
        public float MinValue;
        public float MaxValue;
        public string Unit = "";
        public string Description = "";
    }

    /// <summary>
    /// Airframe-specific PX4 SITL &amp; GNC control parameter table supporting Multirotor, Fixed-Wing, and QuadPlane VTOL.
    /// Synchronizes parameter values across C# closed-loop autopilot, MAVLink PARAM_VALUE/SET, and PX4 startup scripts.
    /// </summary>
    [Serializable]
    public class AirframeControlParameterTable
    {
        // Airframe identification
        public VehicleAirframeCategory Category = VehicleAirframeCategory.Multirotor;
        public string ProfileName = "Standard Quadcopter X-450";
        public string Px4SitlModelName = "iris";
        public int Px4SysAutostart = 10016;
        public int CaAirframe = 0;

        // Airspeed & velocity limits
        public double AirspeedMinMps = 0.0;
        public double AirspeedTrimMps = 10.0;
        public double AirspeedMaxMps = 14.0;
        public double MaxClimbRateMps = 3.5;
        public double MaxSinkRateMps = 2.0;

        // Throttle & trim
        public double ThrottleMin = 0.05;
        public double ThrottleMax = 1.00;
        public double ThrottleTrimOrHover = 0.52;
        public double TrimPitchNormalized = 0.00;
        public double LevelPitchOffsetDeg = 0.0;

        // Outer position & velocity loop gains (Multirotor / VTOL MC)
        public double KpPosXY = 1.25;
        public double KpPosZ = 1.00;
        public double KpVelXY = 3.20;
        public double KiVelXY = 0.45;
        public double KdVelXY = 0.25;
        public double KpVelZ = 5.00;
        public double KiVelZ = 1.20;
        public double KdVelZ = 0.40;

        // Multirotor attitude & rate gains (tuned for 50 Hz Unity lockstep SITL)
        public double McRollPitchP = 4.00;
        public double McYawP = 2.80;
        public double McRollPitchRateP = 0.07;
        public double McRollPitchRateI = 0.05;
        public double McRollPitchRateD = 0.001;
        public double McYawRateP = 0.12;
        public double McYawRateI = 0.04;

        // Fixed-Wing / VTOL FW attitude, rate, TECS & runway takeoff gains
        public double FwRollTimeConstantSec = 0.45;
        public double FwPitchTimeConstantSec = 0.45;
        public double FwRollRateP = 0.22;
        public double FwRollRateI = 0.15;
        public double FwRollRateFF = 0.35;
        public double FwPitchRateP = 0.45;
        public double FwPitchRateI = 0.25;
        public double FwPitchRateFF = 0.40;
        public double FwYawRateP = 0.30;
        public double FwYawRateI = 0.10;
        public double FwYawRateFF = 0.20;
        public double FwMaxRollAngleDeg = 40.0;
        public double FwPitchMinDeg = -20.0;
        public double FwPitchMaxDeg = 25.0;
        public double RunwayTakeoffPitchDeg = 10.0;
        public double NpfgPeriodSec = 10.0;
        public double TecsSpeedWeight = 1.0;

        public List<ControlParameterEntry> Entries = new List<ControlParameterEntry>();

        public static AirframeControlParameterTable CreateForCategory(
            VehicleAirframeCategory category,
            AircraftConfigSpec? config = null,
            BakedAeroDatabase? db = null)
        {
            return category switch
            {
                VehicleAirframeCategory.FixedWing => CreateFixedWingDefault(config, db),
                VehicleAirframeCategory.HybridQuadPlaneVtol => CreateQuadPlaneVtolDefault(config, db),
                VehicleAirframeCategory.TiltRotorVtol => CreateQuadPlaneVtolDefault(config, db),
                _ => CreateMultirotorDefault(config, db)
            };
        }

        public static AirframeControlParameterTable CreateMultirotorDefault(
            AircraftConfigSpec? config = null,
            BakedAeroDatabase? db = null)
        {
            var table = new AirframeControlParameterTable
            {
                Category = VehicleAirframeCategory.Multirotor,
                ProfileName = config != null ? config.AircraftName : "Quadcopter X-450",
                Px4SitlModelName = "iris",
                Px4SysAutostart = 10016,
                CaAirframe = 0,
                AirspeedMinMps = 0.0,
                AirspeedTrimMps = 10.0,
                AirspeedMaxMps = 14.0,
                MaxClimbRateMps = 3.5,
                MaxSinkRateMps = 2.0,
                ThrottleMin = 0.05,
                ThrottleMax = 1.00,
                ThrottleTrimOrHover = (db != null && db.TrimThrottleNormalized > 0.1) ? db.TrimThrottleNormalized : 0.52,
                TrimPitchNormalized = 0.0,
                LevelPitchOffsetDeg = 0.0,
                KpPosXY = 1.25,
                KpPosZ = 1.60,
                KpVelXY = 3.20,
                KiVelXY = 0.45,
                KdVelXY = 0.25,
                KpVelZ = 5.00,
                KiVelZ = 1.20,
                KdVelZ = 0.40,
                McRollPitchP = 6.50,
                McYawP = 2.80,
                McRollPitchRateP = 0.22,
                McRollPitchRateI = 0.08,
                McRollPitchRateD = 0.006,
                McYawRateP = 0.15,
                McYawRateI = 0.04
            };
            table.RebuildEntriesFromFields();
            return table;
        }

        public static AirframeControlParameterTable CreateFixedWingDefault(
            AircraftConfigSpec? config = null,
            BakedAeroDatabase? db = null)
        {
            double designCruise = (config != null && config.DesignCruiseSpeedMps > 5.0) ? config.DesignCruiseSpeedMps : 18.0;
            double trimThr = (db != null && db.TrimThrottleNormalized > 0.15) ? db.TrimThrottleNormalized : 0.50;

            var table = new AirframeControlParameterTable
            {
                Category = VehicleAirframeCategory.FixedWing,
                ProfileName = config != null ? config.AircraftName : "Talon-180 Fixed-Wing UAV",
                Px4SitlModelName = "plane",
                Px4SysAutostart = 1030,
                CaAirframe = 1,
                AirspeedMinMps = 12.0,
                AirspeedTrimMps = designCruise,
                AirspeedMaxMps = 30.0,
                MaxClimbRateMps = 6.0,
                MaxSinkRateMps = 4.0,
                ThrottleMin = 0.05,
                ThrottleMax = 1.00,
                ThrottleTrimOrHover = trimThr,
                TrimPitchNormalized = 0.02,
                LevelPitchOffsetDeg = 2.0,
                KpPosXY = 0.85,
                KpPosZ = 1.20,
                McRollPitchP = 4.50,
                McYawP = 1.50,
                McRollPitchRateP = 0.22,
                McRollPitchRateI = 0.08,
                McRollPitchRateD = 0.004,
                McYawRateP = 0.30,
                McYawRateI = 0.10,
                FwRollTimeConstantSec = 0.45,
                FwPitchTimeConstantSec = 0.45,
                FwRollRateP = 0.22,
                FwRollRateI = 0.15,
                FwRollRateFF = 0.35,
                FwPitchRateP = 0.45,
                FwPitchRateI = 0.25,
                FwPitchRateFF = 0.40,
                FwYawRateP = 0.30,
                FwYawRateI = 0.10,
                FwYawRateFF = 0.20,
                FwMaxRollAngleDeg = 40.0,
                FwPitchMinDeg = -20.0,
                FwPitchMaxDeg = 25.0,
                RunwayTakeoffPitchDeg = 10.0,
                NpfgPeriodSec = 10.0,
                TecsSpeedWeight = 1.0
            };
            table.RebuildEntriesFromFields();
            return table;
        }

        public static AirframeControlParameterTable CreateQuadPlaneVtolDefault(
            AircraftConfigSpec? config = null,
            BakedAeroDatabase? db = null)
        {
            double designCruise = (config != null && config.DesignCruiseSpeedMps > 5.0) ? config.DesignCruiseSpeedMps : 20.0;
            var table = new AirframeControlParameterTable
            {
                Category = (config != null && config.Category == VehicleAirframeCategory.TiltRotorVtol)
                    ? VehicleAirframeCategory.TiltRotorVtol
                    : VehicleAirframeCategory.HybridQuadPlaneVtol,
                ProfileName = config != null ? config.AircraftName : "Millie QuadPlane VTOL",
                Px4SitlModelName = "standard_vtol",
                Px4SysAutostart = 1040,
                CaAirframe = (config != null && config.Category == VehicleAirframeCategory.TiltRotorVtol) ? 3 : 2,
                AirspeedMinMps = 13.0,
                AirspeedTrimMps = designCruise,
                AirspeedMaxMps = 28.0,
                MaxClimbRateMps = 5.0,
                MaxSinkRateMps = 3.0,
                ThrottleMin = 0.05,
                ThrottleMax = 1.00,
                ThrottleTrimOrHover = 0.55,
                TrimPitchNormalized = 0.02,
                LevelPitchOffsetDeg = 2.0,
                KpPosXY = 1.10,
                KpPosZ = 1.50,
                KpVelXY = 2.80,
                KiVelXY = 0.40,
                KdVelXY = 0.20,
                KpVelZ = 4.60,
                KiVelZ = 1.10,
                KdVelZ = 0.35,
                McRollPitchP = 5.50,
                McYawP = 2.20,
                McRollPitchRateP = 0.26,
                McRollPitchRateI = 0.08,
                McRollPitchRateD = 0.006,
                McYawRateP = 0.20,
                McYawRateI = 0.05,
                FwRollTimeConstantSec = 0.50,
                FwPitchTimeConstantSec = 0.50,
                FwRollRateP = 0.25,
                FwRollRateI = 0.15,
                FwRollRateFF = 0.30,
                FwPitchRateP = 0.45,
                FwPitchRateI = 0.25,
                FwPitchRateFF = 0.35,
                FwYawRateP = 0.30,
                FwYawRateI = 0.10,
                FwYawRateFF = 0.20,
                FwMaxRollAngleDeg = 40.0,
                FwPitchMinDeg = -20.0,
                FwPitchMaxDeg = 25.0,
                RunwayTakeoffPitchDeg = 8.0,
                NpfgPeriodSec = 12.0,
                TecsSpeedWeight = 1.0
            };
            table.RebuildEntriesFromFields();
            return table;
        }

        public void RebuildEntriesFromFields()
        {
            Entries.Clear();
            Entries.Add(new ControlParameterEntry { Name = "CA_AIRFRAME", Value = CaAirframe, MinValue = 0f, MaxValue = 3f, Unit = "-", Description = "PX4 Control Allocation Airframe" });

            if (Category == VehicleAirframeCategory.FixedWing ||
                Category == VehicleAirframeCategory.HybridQuadPlaneVtol ||
                Category == VehicleAirframeCategory.TiltRotorVtol)
            {
                Entries.Add(new ControlParameterEntry { Name = "FW_AIRSPD_MIN", Value = (float)AirspeedMinMps, MinValue = 6f, MaxValue = 25f, Unit = "m/s", Description = "Minimum stall safety airspeed" });
                Entries.Add(new ControlParameterEntry { Name = "FW_AIRSPD_TRIM", Value = (float)AirspeedTrimMps, MinValue = 10f, MaxValue = 35f, Unit = "m/s", Description = "Design cruise airspeed" });
                Entries.Add(new ControlParameterEntry { Name = "FW_AIRSPD_MAX", Value = (float)AirspeedMaxMps, MinValue = 15f, MaxValue = 60f, Unit = "m/s", Description = "Maximum allowed airspeed" });
                Entries.Add(new ControlParameterEntry { Name = "FW_THR_MAX", Value = (float)ThrottleMax, MinValue = 0.3f, MaxValue = 1.0f, Unit = "norm", Description = "Max forward cruise throttle" });
                Entries.Add(new ControlParameterEntry { Name = "FW_THR_TRIM", Value = (float)ThrottleTrimOrHover, MinValue = 0.1f, MaxValue = 0.9f, Unit = "norm", Description = "Level cruise trim throttle" });
                Entries.Add(new ControlParameterEntry { Name = "TRIM_PITCH", Value = (float)TrimPitchNormalized, MinValue = -0.3f, MaxValue = 0.3f, Unit = "norm", Description = "Level flight elevator trim offset" });
                Entries.Add(new ControlParameterEntry { Name = "FW_PSP_OFF", Value = (float)LevelPitchOffsetDeg, MinValue = -5f, MaxValue = 10f, Unit = "deg", Description = "Level flight pitch offset" });
                Entries.Add(new ControlParameterEntry { Name = "FW_R_TC", Value = (float)FwRollTimeConstantSec, MinValue = 0.1f, MaxValue = 2.0f, Unit = "s", Description = "Roll attitude time constant" });
                Entries.Add(new ControlParameterEntry { Name = "FW_P_TC", Value = (float)FwPitchTimeConstantSec, MinValue = 0.1f, MaxValue = 2.0f, Unit = "s", Description = "Pitch attitude time constant" });
                Entries.Add(new ControlParameterEntry { Name = "FW_RR_P", Value = (float)FwRollRateP, MinValue = 0.01f, MaxValue = 1.5f, Unit = "-", Description = "Fixed-wing roll rate P" });
                Entries.Add(new ControlParameterEntry { Name = "FW_RR_I", Value = (float)FwRollRateI, MinValue = 0.0f, MaxValue = 1.0f, Unit = "-", Description = "Fixed-wing roll rate I" });
                Entries.Add(new ControlParameterEntry { Name = "FW_RR_FF", Value = (float)FwRollRateFF, MinValue = 0.0f, MaxValue = 1.5f, Unit = "-", Description = "Fixed-wing roll rate FF" });
                Entries.Add(new ControlParameterEntry { Name = "FW_PR_P", Value = (float)FwPitchRateP, MinValue = 0.01f, MaxValue = 2.0f, Unit = "-", Description = "Fixed-wing pitch rate P" });
                Entries.Add(new ControlParameterEntry { Name = "FW_PR_I", Value = (float)FwPitchRateI, MinValue = 0.0f, MaxValue = 1.0f, Unit = "-", Description = "Fixed-wing pitch rate I" });
                Entries.Add(new ControlParameterEntry { Name = "FW_PR_FF", Value = (float)FwPitchRateFF, MinValue = 0.0f, MaxValue = 1.5f, Unit = "-", Description = "Fixed-wing pitch rate FF" });
                Entries.Add(new ControlParameterEntry { Name = "FW_YR_P", Value = (float)FwYawRateP, MinValue = 0.0f, MaxValue = 1.5f, Unit = "-", Description = "Fixed-wing yaw rate P" });
                Entries.Add(new ControlParameterEntry { Name = "FW_YR_I", Value = (float)FwYawRateI, MinValue = 0.0f, MaxValue = 1.0f, Unit = "-", Description = "Fixed-wing yaw rate I" });
                Entries.Add(new ControlParameterEntry { Name = "FW_YR_FF", Value = (float)FwYawRateFF, MinValue = 0.0f, MaxValue = 1.0f, Unit = "-", Description = "Fixed-wing yaw rate FF" });
                Entries.Add(new ControlParameterEntry { Name = "FW_R_LIM", Value = (float)FwMaxRollAngleDeg, MinValue = 15f, MaxValue = 70f, Unit = "deg", Description = "Max fixed-wing roll angle" });
                Entries.Add(new ControlParameterEntry { Name = "FW_T_CLMB_MAX", Value = (float)MaxClimbRateMps, MinValue = 1f, MaxValue = 15f, Unit = "m/s", Description = "Max TECS climb rate" });
                Entries.Add(new ControlParameterEntry { Name = "RWTO_PSP", Value = (float)RunwayTakeoffPitchDeg, MinValue = 2f, MaxValue = 20f, Unit = "deg", Description = "Runway takeoff pitch setpoint" });
                Entries.Add(new ControlParameterEntry { Name = "NPFG_PERIOD", Value = (float)NpfgPeriodSec, MinValue = 4f, MaxValue = 25f, Unit = "s", Description = "NPFG / L1 guidance period" });
            }

            if (Category == VehicleAirframeCategory.Multirotor ||
                Category == VehicleAirframeCategory.HybridQuadPlaneVtol ||
                Category == VehicleAirframeCategory.TiltRotorVtol)
            {
                Entries.Add(new ControlParameterEntry { Name = "MPC_XY_CRUISE", Value = (float)AirspeedTrimMps, MinValue = 2f, MaxValue = 25f, Unit = "m/s", Description = "Multirotor cruise speed" });
                Entries.Add(new ControlParameterEntry { Name = "MPC_XY_VEL_MAX", Value = (float)AirspeedMaxMps, MinValue = 3f, MaxValue = 30f, Unit = "m/s", Description = "Multirotor max horizontal speed" });
                Entries.Add(new ControlParameterEntry { Name = "MPC_Z_VEL_MAX_UP", Value = (float)MaxClimbRateMps, MinValue = 1f, MaxValue = 10f, Unit = "m/s", Description = "Multirotor max ascent rate" });
                Entries.Add(new ControlParameterEntry { Name = "MPC_THR_HOVER", Value = (float)ThrottleTrimOrHover, MinValue = 0.2f, MaxValue = 0.85f, Unit = "norm", Description = "Multirotor hover throttle" });
                Entries.Add(new ControlParameterEntry { Name = "MPC_THR_MAX", Value = (float)ThrottleMax, MinValue = 0.5f, MaxValue = 1.0f, Unit = "norm", Description = "Multirotor max throttle" });
                Entries.Add(new ControlParameterEntry { Name = "MPC_XY_P", Value = (float)KpPosXY, MinValue = 0.2f, MaxValue = 4.0f, Unit = "-", Description = "Horizontal position P gain" });
                Entries.Add(new ControlParameterEntry { Name = "MPC_Z_P", Value = (float)KpPosZ, MinValue = 0.2f, MaxValue = 4.0f, Unit = "-", Description = "Vertical position P gain" });
                Entries.Add(new ControlParameterEntry { Name = "MC_ROLL_P", Value = (float)McRollPitchP, MinValue = 1.0f, MaxValue = 12.0f, Unit = "-", Description = "Multirotor roll/pitch attitude P" });
                Entries.Add(new ControlParameterEntry { Name = "MC_YAW_P", Value = (float)McYawP, MinValue = 0.5f, MaxValue = 8.0f, Unit = "-", Description = "Multirotor yaw attitude P" });
                Entries.Add(new ControlParameterEntry { Name = "MC_ROLLRATE_P", Value = (float)McRollPitchRateP, MinValue = 0.02f, MaxValue = 0.8f, Unit = "-", Description = "Multirotor roll/pitch rate P" });
                Entries.Add(new ControlParameterEntry { Name = "MC_YAWRATE_P", Value = (float)McYawRateP, MinValue = 0.02f, MaxValue = 0.8f, Unit = "-", Description = "Multirotor yaw rate P" });
            }
        }

        public void SyncFieldsFromEntries()
        {
            for (int i = 0; i < Entries.Count; i++)
            {
                InternalApplySingleParameter(Entries[i].Name, Entries[i].Value);
            }
        }

        public bool TrySetParameter(string name, float value)
        {
            bool matched = InternalApplySingleParameter(name, value);
            if (!matched)
                return false;

            bool updatedEntry = false;
            for (int i = 0; i < Entries.Count; i++)
            {
                if (string.Equals(Entries[i].Name, name, StringComparison.Ordinal))
                {
                    Entries[i].Value = value;
                    updatedEntry = true;
                }
            }

            if (!updatedEntry)
                RebuildEntriesFromFields();

            return true;
        }

        public List<(string Name, float Value)> GetMavlinkParameterList()
        {
            if (Entries.Count == 0)
                RebuildEntriesFromFields();

            var list = new List<(string Name, float Value)>(Entries.Count + 4);
            var seen = new HashSet<string>(StringComparer.Ordinal);

            for (int i = 0; i < Entries.Count; i++)
            {
                var e = Entries[i];
                if (seen.Add(e.Name))
                    list.Add((e.Name, e.Value));
            }

            // Include common MAVLink aliases expected by QGC & tests
            if (seen.Add("MPC_XY_P")) list.Add(("MPC_XY_P", (float)KpPosXY));
            if (seen.Add("MPC_Z_P")) list.Add(("MPC_Z_P", (float)KpPosZ));
            if (seen.Add("MPC_XY_VEL_P")) list.Add(("MPC_XY_VEL_P", (float)KpVelXY));
            if (seen.Add("MPC_Z_VEL_P")) list.Add(("MPC_Z_VEL_P", (float)KpVelZ));
            if (seen.Add("MC_ROLL_P")) list.Add(("MC_ROLL_P", (float)McRollPitchP));
            if (seen.Add("MC_ROLLRATE_P")) list.Add(("MC_ROLLRATE_P", (float)McRollPitchRateP));

            return list;
        }

        public string GeneratePx4ParameterScript(AircraftConfigSpec config, BakedAeroDatabase db)
        {
            var sb = new StringBuilder();
            var inv = CultureInfo.InvariantCulture;

            sb.AppendLine("# BEGIN_UNITY_STAGE1_CA_PARAMS");
            sb.AppendLine($"# Auto-generated PX4 Parameter Table for: {config.AircraftName} ({Category} / {Px4SitlModelName})");
            sb.AppendLine($"# Generated by FlightDynamics 6-DOF Engineering Suite (Apache-2.0)");
            sb.AppendLine($"# Mass: {config.MassKg.ToString("F3", inv)} kg | Static Margin: {(db.StaticMarginFraction * 100.0).ToString("F1", inv)}% MAC | NP_x: {db.NeutralPointXFrd.ToString("F3", inv)} m");
            sb.AppendLine();

            sb.AppendLine($"param set CA_AIRFRAME {CaAirframe}");
            sb.AppendLine($"param set CA_ROTOR_COUNT {config.Rotors.Count}");

            for (int i = 0; i < config.Rotors.Count; i++)
            {
                var r = config.Rotors[i];
                double kt = (i < db.RotorLumpedKt.Length) ? db.RotorLumpedKt[i] : 1.5e-5;
                double kq = (i < db.RotorLumpedKq.Length) ? db.RotorLumpedKq[i] : 2.5e-7;
                double omegaMax = r.MaxRpm * FlightDynamics6DoF.Core.Math.MathConstants.RpmToRadPerSec;
                // PX4 CA_ROTOR*_CT is max thrust in Newtons at u=1 (Thrust = CT * u^2), not N/(rad/s)^2
                double ctNewton = System.Math.Clamp(kt * omegaMax * omegaMax, 0.5, 100.0);
                // PX4 CA_ROTOR*_KM requires >0 for CCW (RotationDirection=-1) and <0 for CW (+1)
                double km = -(kq / System.Math.Max(1e-7, kt)) * r.RotationDirection;

                sb.AppendLine($"param set CA_ROTOR{i}_PX {r.MountPositionFrd.X.ToString("F4", inv)}");
                sb.AppendLine($"param set CA_ROTOR{i}_PY {r.MountPositionFrd.Y.ToString("F4", inv)}");
                sb.AppendLine($"param set CA_ROTOR{i}_PZ {r.MountPositionFrd.Z.ToString("F4", inv)}");
                sb.AppendLine($"param set CA_ROTOR{i}_AX {r.ThrustAxisFrd.X.ToString("F4", inv)}");
                sb.AppendLine($"param set CA_ROTOR{i}_AY {r.ThrustAxisFrd.Y.ToString("F4", inv)}");
                sb.AppendLine($"param set CA_ROTOR{i}_AZ {r.ThrustAxisFrd.Z.ToString("F4", inv)}");
                sb.AppendLine($"param set CA_ROTOR{i}_CT {ctNewton.ToString("F4", inv)}");
                sb.AppendLine($"param set CA_ROTOR{i}_KM {km.ToString("F4", inv)}");
            }

            if (config.Wings != null && config.Wings.Count > 0)
            {
                sb.AppendLine($"param set CA_SV_CS_COUNT {config.Wings.Count}");
            }

            sb.AppendLine("param set IMU_INTEG_RATE 400");
            sb.AppendLine("param set SENS_EN_GPSSIM 1");
            sb.AppendLine("param set SENS_EN_BAROSIM 1");
            sb.AppendLine("param set SENS_EN_MAGSIM 1");
            sb.AppendLine("param set SYS_HAS_BARO 1");
            sb.AppendLine("param set SYS_HAS_MAG 1");

            if (Category == VehicleAirframeCategory.FixedWing ||
                Category == VehicleAirframeCategory.HybridQuadPlaneVtol ||
                Category == VehicleAirframeCategory.TiltRotorVtol)
            {
                sb.AppendLine();
                sb.AppendLine("# Fixed-Wing / VTOL Fixed-Wing Flight Control & Sign Alignment");
                sb.AppendLine("param set PWM_MAIN_REV 0");
                sb.AppendLine("param set CBRK_AIRSPD_CHK 162128");
                sb.AppendLine($"param set FW_AIRSPD_MIN {AirspeedMinMps.ToString("F2", inv)}");
                sb.AppendLine($"param set FW_AIRSPD_TRIM {AirspeedTrimMps.ToString("F2", inv)}");
                sb.AppendLine($"param set FW_AIRSPD_MAX {AirspeedMaxMps.ToString("F2", inv)}");
                sb.AppendLine($"param set FW_THR_MIN {ThrottleMin.ToString("F2", inv)}");
                sb.AppendLine($"param set FW_THR_MAX {ThrottleMax.ToString("F2", inv)}");
                sb.AppendLine($"param set FW_THR_TRIM {ThrottleTrimOrHover.ToString("F2", inv)}");
                sb.AppendLine($"param set TRIM_PITCH {TrimPitchNormalized.ToString("F3", inv)}");
                sb.AppendLine($"param set FW_PSP_OFF {LevelPitchOffsetDeg.ToString("F2", inv)}");
                sb.AppendLine($"param set FW_R_TC {FwRollTimeConstantSec.ToString("F2", inv)}");
                sb.AppendLine($"param set FW_P_TC {FwPitchTimeConstantSec.ToString("F2", inv)}");
                sb.AppendLine($"param set FW_RR_P {FwRollRateP.ToString("F3", inv)}");
                sb.AppendLine($"param set FW_RR_I {FwRollRateI.ToString("F3", inv)}");
                sb.AppendLine($"param set FW_RR_FF {FwRollRateFF.ToString("F3", inv)}");
                sb.AppendLine($"param set FW_PR_P {FwPitchRateP.ToString("F3", inv)}");
                sb.AppendLine($"param set FW_PR_I {FwPitchRateI.ToString("F3", inv)}");
                sb.AppendLine($"param set FW_PR_FF {FwPitchRateFF.ToString("F3", inv)}");
                sb.AppendLine($"param set FW_YR_P {FwYawRateP.ToString("F3", inv)}");
                sb.AppendLine($"param set FW_YR_I {FwYawRateI.ToString("F3", inv)}");
                sb.AppendLine($"param set FW_YR_FF {FwYawRateFF.ToString("F3", inv)}");
                sb.AppendLine($"param set FW_R_LIM {FwMaxRollAngleDeg.ToString("F1", inv)}");
                sb.AppendLine($"param set FW_P_LIM_MIN {FwPitchMinDeg.ToString("F1", inv)}");
                sb.AppendLine($"param set FW_P_LIM_MAX {FwPitchMaxDeg.ToString("F1", inv)}");
                sb.AppendLine($"param set FW_T_CLMB_MAX {MaxClimbRateMps.ToString("F2", inv)}");
                sb.AppendLine($"param set FW_T_SINK_MAX {MaxSinkRateMps.ToString("F2", inv)}");
                sb.AppendLine($"param set FW_T_SPDWEIGHT {TecsSpeedWeight.ToString("F2", inv)}");
                sb.AppendLine($"param set NPFG_PERIOD {NpfgPeriodSec.ToString("F2", inv)}");

                if (Category == VehicleAirframeCategory.FixedWing)
                {
                    sb.AppendLine("param set RWTO_TKOFF 1");
                    sb.AppendLine($"param set RWTO_PSP {RunwayTakeoffPitchDeg.ToString("F1", inv)}");
                    sb.AppendLine("param set RWTO_MAX_THR 1.00");
                    sb.AppendLine("param set RWTO_AIRSPD_SCL 1.15");
                }
            }

            if (Category == VehicleAirframeCategory.Multirotor ||
                Category == VehicleAirframeCategory.HybridQuadPlaneVtol ||
                Category == VehicleAirframeCategory.TiltRotorVtol)
            {
                sb.AppendLine();
                sb.AppendLine("# Multirotor / VTOL Hover Control Gains");
                sb.AppendLine($"param set MPC_XY_CRUISE {AirspeedTrimMps.ToString("F2", inv)}");
                sb.AppendLine($"param set MPC_XY_VEL_MAX {AirspeedMaxMps.ToString("F2", inv)}");
                sb.AppendLine($"param set MPC_Z_VEL_MAX_UP {MaxClimbRateMps.ToString("F2", inv)}");
                sb.AppendLine($"param set MPC_Z_VEL_MAX_DN {MaxSinkRateMps.ToString("F2", inv)}");
                sb.AppendLine($"param set MPC_THR_MIN {ThrottleMin.ToString("F2", inv)}");
                sb.AppendLine($"param set MPC_THR_MAX {ThrottleMax.ToString("F2", inv)}");
                sb.AppendLine($"param set MPC_THR_HOVER {ThrottleTrimOrHover.ToString("F2", inv)}");
                sb.AppendLine($"param set MPC_XY_P {KpPosXY.ToString("F3", inv)}");
                sb.AppendLine($"param set MPC_Z_P {KpPosZ.ToString("F3", inv)}");
                sb.AppendLine($"param set MC_ROLL_P {McRollPitchP.ToString("F3", inv)}");
                sb.AppendLine($"param set MC_PITCH_P {McRollPitchP.ToString("F3", inv)}");
                sb.AppendLine($"param set MC_YAW_P {McYawP.ToString("F3", inv)}");
                sb.AppendLine($"param set MC_ROLLRATE_P {McRollPitchRateP.ToString("F3", inv)}");
                sb.AppendLine($"param set MC_ROLLRATE_I {McRollPitchRateI.ToString("F3", inv)}");
                sb.AppendLine($"param set MC_ROLLRATE_D {McRollPitchRateD.ToString("F3", inv)}");
                sb.AppendLine($"param set MC_PITCHRATE_P {McRollPitchRateP.ToString("F3", inv)}");
                sb.AppendLine($"param set MC_PITCHRATE_I {McRollPitchRateI.ToString("F3", inv)}");
                sb.AppendLine($"param set MC_PITCHRATE_D {McRollPitchRateD.ToString("F3", inv)}");
                sb.AppendLine($"param set MC_YAWRATE_P {McYawRateP.ToString("F3", inv)}");
            }

            sb.AppendLine("# END_UNITY_STAGE1_CA_PARAMS");
            return sb.ToString();
        }

        private bool InternalApplySingleParameter(string name, float val)
        {
            switch (name)
            {
                case "CA_AIRFRAME": CaAirframe = (int)System.Math.Round(val); return true;
                case "FW_AIRSPD_MIN": AirspeedMinMps = val; return true;
                case "FW_AIRSPD_TRIM":
                case "MPC_XY_CRUISE": AirspeedTrimMps = val; return true;
                case "FW_AIRSPD_MAX":
                case "MPC_XY_VEL_MAX": AirspeedMaxMps = val; return true;
                case "FW_T_CLMB_MAX":
                case "MPC_Z_VEL_MAX_UP": MaxClimbRateMps = val; return true;
                case "FW_T_SINK_MAX":
                case "MPC_Z_VEL_MAX_DN": MaxSinkRateMps = val; return true;
                case "FW_THR_MIN":
                case "MPC_THR_MIN": ThrottleMin = val; return true;
                case "FW_THR_MAX":
                case "MPC_THR_MAX": ThrottleMax = val; return true;
                case "FW_THR_TRIM":
                case "MPC_THR_HOVER": ThrottleTrimOrHover = val; return true;
                case "TRIM_PITCH": TrimPitchNormalized = val; return true;
                case "FW_PSP_OFF": LevelPitchOffsetDeg = val; return true;
                case "MPC_XY_P": KpPosXY = val; return true;
                case "MPC_Z_P": KpPosZ = val; return true;
                case "MPC_XY_VEL_P":
                case "MPC_XY_VEL_P_ACC": KpVelXY = val; return true;
                case "MPC_Z_VEL_P":
                case "MPC_Z_VEL_P_ACC": KpVelZ = val; return true;
                case "MC_ROLL_P":
                case "MC_PITCH_P": McRollPitchP = val; return true;
                case "MC_YAW_P": McYawP = val; return true;
                case "MC_ROLLRATE_P":
                case "MC_PITCHRATE_P": McRollPitchRateP = val; return true;
                case "MC_YAWRATE_P": McYawRateP = val; return true;
                case "FW_R_TC": FwRollTimeConstantSec = val; return true;
                case "FW_P_TC": FwPitchTimeConstantSec = val; return true;
                case "FW_RR_P": FwRollRateP = val; return true;
                case "FW_RR_I": FwRollRateI = val; return true;
                case "FW_RR_FF": FwRollRateFF = val; return true;
                case "FW_PR_P": FwPitchRateP = val; return true;
                case "FW_PR_I": FwPitchRateI = val; return true;
                case "FW_PR_FF": FwPitchRateFF = val; return true;
                case "FW_YR_P": FwYawRateP = val; return true;
                case "FW_YR_I": FwYawRateI = val; return true;
                case "FW_YR_FF": FwYawRateFF = val; return true;
                case "FW_R_LIM": FwMaxRollAngleDeg = val; return true;
                case "FW_P_LIM_MIN": FwPitchMinDeg = val; return true;
                case "FW_P_LIM_MAX": FwPitchMaxDeg = val; return true;
                case "RWTO_PSP": RunwayTakeoffPitchDeg = val; return true;
                case "NPFG_PERIOD": NpfgPeriodSec = val; return true;
                case "FW_T_SPDWEIGHT": TecsSpeedWeight = val; return true;
                default: return false;
            }
        }
    }
}
