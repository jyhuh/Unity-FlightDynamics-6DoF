// Licensed under the Apache License, Version 2.0.
#nullable enable
using System;
using System.Collections.Generic;
using System.Linq;
using FlightDynamics6DoF.Core.Dynamics;
using FlightDynamics6DoF.Core.Specs;
using UnityEngine;

namespace FlightDynamics6DoF.Editor
{
    /// <summary>
    /// Immutable descriptor for a single plottable telemetry channel.
    /// </summary>
    internal sealed class TelemetryChannelDef
    {
        public string Name { get; }
        public Color Color { get; }
        public Func<TelemetryLogFrame, double> Extractor { get; }

        public TelemetryChannelDef(string name, Color color, Func<TelemetryLogFrame, double> extractor)
        {
            Name = name;
            Color = color;
            Extractor = extractor;
        }
    }

    /// <summary>
    /// Unified chart specification rendered by <see cref="TelemetryChartRenderer"/>.
    /// Used uniformly by both built-in Command-vs-Response presets and user custom charts.
    /// </summary>
    internal sealed class TelemetryChartDefinition
    {
        public string Title { get; }
        public IReadOnlyList<TelemetryChannelDef> Series { get; }

        public TelemetryChartDefinition(string title, IReadOnlyList<TelemetryChannelDef> series)
        {
            Title = title;
            Series = series;
        }
    }

    /// <summary>
    /// Mutable configuration for a user-created custom chart.
    /// </summary>
    internal sealed class TelemetryCustomChart
    {
        public string Title = "Custom Graph";
        public readonly List<int> ChannelIndices = new List<int> { 0, 1 };
        public int PendingAddIndex = 0;

        public TelemetryChartDefinition ToDefinition(IReadOnlyList<TelemetryChannelDef> catalog)
        {
            var series = new List<TelemetryChannelDef>(ChannelIndices.Count);
            for (int i = 0; i < ChannelIndices.Count; i++)
            {
                int idx = ChannelIndices[i];
                if (idx >= 0 && idx < catalog.Count)
                    series.Add(catalog[idx]);
            }
            return new TelemetryChartDefinition(Title, series);
        }
    }

    /// <summary>
    /// Central registry of all plottable telemetry channels and built-in Command-vs-Response presets.
    /// </summary>
    internal static class TelemetryChannelRegistry
    {
        public static readonly TelemetryChannelDef[] Channels =
        {
            new TelemetryChannelDef("Altitude AGL [m]", new Color(0.30f, 0.90f, 0.40f), f => f.AltitudeAgl),
            new TelemetryChannelDef("Target Alt [m]", Color.gray, f => f.TargetAltAgl),
            new TelemetryChannelDef("Roll [deg]", new Color(1.00f, 0.35f, 0.35f), f => f.RollDeg),
            new TelemetryChannelDef("Target Roll [deg]", new Color(1.00f, 0.72f, 0.72f), f => f.TargetRollDeg),
            new TelemetryChannelDef("Pitch [deg]", new Color(0.35f, 0.70f, 1.00f), f => f.PitchDeg),
            new TelemetryChannelDef("Target Pitch [deg]", new Color(0.95f, 0.75f, 0.25f), f => f.TargetPitchDeg),
            new TelemetryChannelDef("Yaw [deg]", new Color(0.30f, 0.90f, 0.45f), f => f.YawDeg),
            new TelemetryChannelDef("Target Yaw [deg]", new Color(0.72f, 0.95f, 0.72f), f => f.TargetYawDeg),
            new TelemetryChannelDef("Airspeed [m/s]", new Color(0.45f, 0.80f, 1.00f), f => f.AirspeedMps),
            new TelemetryChannelDef("Target Airspeed [m/s]", Color.gray, f => f.TargetAirspeedMps),
            new TelemetryChannelDef("Climb -VelD [m/s]", new Color(0.35f, 0.70f, 1.00f), f => -f.VelD),
            new TelemetryChannelDef("Pos N [m]", new Color(0.95f, 0.55f, 0.35f), f => f.PosN),
            new TelemetryChannelDef("Pos E [m]", new Color(0.35f, 0.85f, 0.95f), f => f.PosE),
            new TelemetryChannelDef("Vel N [m/s]", new Color(0.95f, 0.70f, 0.40f), f => f.VelN),
            new TelemetryChannelDef("Vel E [m/s]", new Color(0.45f, 0.90f, 0.95f), f => f.VelE),
            new TelemetryChannelDef("Alpha [deg]", new Color(0.85f, 0.55f, 1.00f), f => f.AlphaDeg),
            new TelemetryChannelDef("Beta [deg]", new Color(1.00f, 0.65f, 0.30f), f => f.BetaDeg),
            new TelemetryChannelDef("p [deg/s]", new Color(1.00f, 0.45f, 0.45f), f => f.P_Degps),
            new TelemetryChannelDef("q [deg/s]", new Color(0.45f, 0.80f, 1.00f), f => f.Q_Degps),
            new TelemetryChannelDef("r [deg/s]", new Color(0.45f, 0.95f, 0.55f), f => f.R_Degps),
            new TelemetryChannelDef("Mx [Nm]", new Color(0.85f, 0.55f, 1.00f), f => f.MxNm),
            new TelemetryChannelDef("My [Nm]", new Color(1.00f, 0.65f, 0.30f), f => f.MyNm),
            new TelemetryChannelDef("Mz [Nm]", new Color(0.95f, 0.90f, 0.35f), f => f.MzNm),
            new TelemetryChannelDef("Total Thrust [N]", new Color(0.35f, 0.95f, 0.65f), f => f.TotalThrustNewtons),
            new TelemetryChannelDef("Battery [V]", new Color(0.95f, 0.80f, 0.30f), f => f.BatteryVoltage),
            new TelemetryChannelDef("Battery [A]", new Color(1.00f, 0.45f, 0.35f), f => f.BatteryCurrent),
            new TelemetryChannelDef("Act0", new Color(0.30f, 0.90f, 0.45f), f => f.Act0),
            new TelemetryChannelDef("Act1", new Color(1.00f, 0.45f, 0.45f), f => f.Act1),
            new TelemetryChannelDef("Act2", new Color(0.35f, 0.70f, 1.00f), f => f.Act2),
            new TelemetryChannelDef("Act3", new Color(0.95f, 0.75f, 0.25f), f => f.Act3),
            new TelemetryChannelDef("Act4 (Thr)", new Color(0.30f, 0.90f, 0.45f), f => f.Act4),
            new TelemetryChannelDef("Act5 (Ail)", new Color(1.00f, 0.35f, 0.35f), f => f.Act5),
            new TelemetryChannelDef("Act7 (Elev)", new Color(0.35f, 0.70f, 1.00f), f => f.Act7),
            new TelemetryChannelDef("Act8 (Rud)", new Color(0.85f, 0.55f, 1.00f), f => f.Act8)
        };

        public static readonly string[] ChannelNames = Channels.Select(c => c.Name).ToArray();

        private static readonly TelemetryChartDefinition[] CommonPresets =
        {
            new TelemetryChartDefinition(
                "1. Command vs Response: Altitude & Climb Rate [m, m/s]",
                new[] { Channels[0], Channels[1], Channels[10] }),
            new TelemetryChartDefinition(
                "2. Command vs Response: Roll & Pitch [deg]",
                new[] { Channels[2], Channels[3], Channels[4], Channels[5] }),
            new TelemetryChartDefinition(
                "3. Command vs Response: Yaw / Heading [deg]",
                new[] { Channels[6], Channels[7] }),
            new TelemetryChartDefinition(
                "4. Command vs Response: Airspeed & Aero Angles [m/s, deg]",
                new[] { Channels[8], Channels[9], Channels[15], Channels[16] }),
            new TelemetryChartDefinition(
                "5. Body Rates & Aerodynamic Moments [deg/s, Nm]",
                new[] { Channels[17], Channels[18], Channels[19], Channels[20], Channels[21] })
        };

        private static readonly TelemetryChartDefinition MultirotorActuatorChart = new TelemetryChartDefinition(
            "6. Normalized Actuator Outputs (Multirotor Motors 1-4) [0..1]",
            new[] { Channels[26], Channels[27], Channels[28], Channels[29] });

        private static readonly TelemetryChartDefinition FixedWingActuatorChart = new TelemetryChartDefinition(
            "6. Normalized Actuator Outputs (Fixed-Wing) [-1..1]",
            new[] { Channels[30], Channels[31], Channels[32], Channels[33] });

        private static readonly TelemetryChartDefinition VtolActuatorChart = new TelemetryChartDefinition(
            "6. Normalized Actuator Outputs (VTOL Motors & Surfaces) [-1..1]",
            new[] { Channels[26], Channels[27], Channels[28], Channels[29], Channels[30], Channels[31], Channels[32], Channels[33] });

        public static TelemetryChartDefinition[] GetBuiltInPresets(VehicleAirframeCategory category)
        {
            var actuatorChart = category switch
            {
                VehicleAirframeCategory.FixedWing => FixedWingActuatorChart,
                VehicleAirframeCategory.HybridQuadPlaneVtol => VtolActuatorChart,
                VehicleAirframeCategory.TiltRotorVtol => VtolActuatorChart,
                _ => MultirotorActuatorChart
            };

            var presets = new TelemetryChartDefinition[CommonPresets.Length + 1];
            Array.Copy(CommonPresets, presets, CommonPresets.Length);
            presets[CommonPresets.Length] = actuatorChart;
            return presets;
        }
    }
}
