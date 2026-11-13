// Licensed under the Apache License, Version 2.0.
using System;
using System.Collections.Generic;
using System.Globalization;
using System.IO;
using System.Text;
using FlightDynamics6DoF.Core.Gnc;
using FlightDynamics6DoF.Core.Math;
using FlightDynamics6DoF.Core.Specs;

namespace FlightDynamics6DoF.Core.Dynamics
{
    [Serializable]
    public struct TelemetryLogFrame
    {
        public double SimTimeSec;
        public VehicleAirframeCategory Category;
        public SimulationFidelityLevel Fidelity;
        public bool IsArmed;
        public bool IsGrounded;
        public double PosN, PosE, PosD, AltitudeAgl;
        public double TargetPosN, TargetPosE, TargetAltAgl;
        public double VelN, VelE, VelD, AirspeedMps, TargetAirspeedMps;
        public double AlphaDeg, BetaDeg;
        public double RollDeg, PitchDeg, YawDeg;
        public double TargetRollDeg, TargetPitchDeg, TargetYawDeg;
        public double P_Degps, Q_Degps, R_Degps;
        public double FxFrd, FyFrd, FzFrd, MxNm, MyNm, MzNm;
        public double Act0, Act1, Act2, Act3, Act4, Act5, Act6, Act7, Act8;
        public double TotalThrustNewtons, BatteryVoltage, BatteryCurrent;
    }

    [Serializable]
    public struct FlightPerformanceMetrics
    {
        public int SampleCount;
        public double DurationSec;
        public double AltitudeRmseMeters;
        public double AltitudeMaxErrorMeters;
        public double AirspeedRmseMps;
        public double RollRmseDeg;
        public double PitchRmseDeg;
        public double MaxAltitudeAglMeters;
        public double MaxAirspeedMps;
        public double MaxRollAbsDeg;
    }

    /// <summary>
    /// High-frequency academic telemetry recorder, quantitative RMSE performance analyzer,
    /// CSV exporter (MATLAB/Python compatible), and self-contained interactive HTML5 + SVG vector graph report generator.
    /// </summary>
    public class FlightTelemetryLogger
    {
        private readonly List<TelemetryLogFrame> _fdFrames = new List<TelemetryLogFrame>();
        private double _fdLastSampleSimTime = -1e9;

        public bool IsRecording { get; set; } = true;
        public double SampleRateHz { get; set; } = 50.0;
        public int MaxFrames { get; set; } = 30000;

        public IReadOnlyList<TelemetryLogFrame> Frames => _fdFrames;

        public void StartRecording()
        {
            IsRecording = true;
        }

        public void PauseRecording()
        {
            IsRecording = false;
        }

        public void Clear()
        {
            _fdFrames.Clear();
            _fdLastSampleSimTime = -1e9;
        }

        public void EnforceMaxFrames()
        {
            if (MaxFrames <= 0) return;
            if (_fdFrames.Count > MaxFrames)
            {
                _fdFrames.RemoveRange(0, _fdFrames.Count - MaxFrames);
            }
        }

        public void RecordFrame(TelemetryLogFrame frame)
        {
            if (_fdFrames.Count >= MaxFrames && MaxFrames > 0)
            {
                _fdFrames.RemoveAt(0);
            }
            _fdFrames.Add(frame);
            _fdLastSampleSimTime = frame.SimTimeSec;
        }

        public void RecordStep(MultiFidelitySixDofEngine engine, CascadedFlightController? controller, bool forceSample = false)
        {
            if (engine == null) return;
            if (!IsRecording && !forceSample) return;

            var st = engine.State;
            double minPeriod = 1.0 / System.Math.Clamp(SampleRateHz, 1.0, 1000.0);

            if (!forceSample && _fdFrames.Count > 0)
            {
                if (st.SimTimeSec >= _fdLastSampleSimTime && (st.SimTimeSec - _fdLastSampleSimTime) < minPeriod - 1e-6)
                    return;
            }

            Vec3d eulerDeg = st.AttitudeBodyToNed.ToEulerZyx() * FrameConverter.Rad2Deg;
            Vec3d tgtEulerDeg = controller != null
                ? controller.TargetAttitudeBodyToNed.ToEulerZyx() * FrameConverter.Rad2Deg
                : Vec3d.Zero;

            double totalThrustN = 0.0;
            if (st.RotorThrustNewtons != null)
            {
                for (int i = 0; i < st.RotorThrustNewtons.Length; i++)
                    totalThrustN += st.RotorThrustNewtons[i];
            }

            double[] acts = st.ActuatorControlsNormalized;
            double tgtAlt = controller != null ? System.Math.Max(0.0, -controller.TargetPositionNed.Z) : 0.0;
            double tgtAirspeed = controller != null ? controller.ParameterTable.AirspeedTrimMps : engine.Config.DesignCruiseSpeedMps;

            var frame = new TelemetryLogFrame
            {
                SimTimeSec = st.SimTimeSec,
                Category = engine.Config.Category,
                Fidelity = engine.FidelityLevel,
                IsArmed = controller != null && controller.IsArmed,
                IsGrounded = st.IsGrounded,
                PosN = st.PositionNed.X,
                PosE = st.PositionNed.Y,
                PosD = st.PositionNed.Z,
                AltitudeAgl = st.AltitudeAglMeters,
                TargetPosN = controller != null ? controller.TargetPositionNed.X : 0.0,
                TargetPosE = controller != null ? controller.TargetPositionNed.Y : 0.0,
                TargetAltAgl = tgtAlt,
                VelN = st.VelocityNed.X,
                VelE = st.VelocityNed.Y,
                VelD = st.VelocityNed.Z,
                AirspeedMps = st.AirspeedMps,
                TargetAirspeedMps = tgtAirspeed,
                AlphaDeg = st.AlphaRad * FrameConverter.Rad2Deg,
                BetaDeg = st.BetaRad * FrameConverter.Rad2Deg,
                RollDeg = eulerDeg.X,
                PitchDeg = eulerDeg.Y,
                YawDeg = eulerDeg.Z,
                TargetRollDeg = tgtEulerDeg.X,
                TargetPitchDeg = tgtEulerDeg.Y,
                TargetYawDeg = controller != null ? controller.TargetYawRad * FrameConverter.Rad2Deg : 0.0,
                P_Degps = st.AngularVelocityBodyFrd.X * FrameConverter.Rad2Deg,
                Q_Degps = st.AngularVelocityBodyFrd.Y * FrameConverter.Rad2Deg,
                R_Degps = st.AngularVelocityBodyFrd.Z * FrameConverter.Rad2Deg,
                FxFrd = st.TotalAeroPropForceFrd.X,
                FyFrd = st.TotalAeroPropForceFrd.Y,
                FzFrd = st.TotalAeroPropForceFrd.Z,
                MxNm = st.TotalAeroPropMomentCgFrd.X,
                MyNm = st.TotalAeroPropMomentCgFrd.Y,
                MzNm = st.TotalAeroPropMomentCgFrd.Z,
                Act0 = acts.Length > 0 ? acts[0] : 0.0,
                Act1 = acts.Length > 1 ? acts[1] : 0.0,
                Act2 = acts.Length > 2 ? acts[2] : 0.0,
                Act3 = acts.Length > 3 ? acts[3] : 0.0,
                Act4 = acts.Length > 4 ? acts[4] : 0.0,
                Act5 = acts.Length > 5 ? acts[5] : 0.0,
                Act6 = acts.Length > 6 ? acts[6] : 0.0,
                Act7 = acts.Length > 7 ? acts[7] : 0.0,
                Act8 = acts.Length > 8 ? acts[8] : 0.0,
                TotalThrustNewtons = totalThrustN,
                BatteryVoltage = st.BatteryVoltageVolts,
                BatteryCurrent = st.BatteryCurrentAmps
            };

            RecordFrame(frame);
        }

        public FlightPerformanceMetrics ComputeMetrics()
        {
            var m = new FlightPerformanceMetrics();
            int n = _fdFrames.Count;
            if (n == 0) return m;

            m.SampleCount = n;
            m.DurationSec = System.Math.Max(0.0, _fdFrames[n - 1].SimTimeSec - _fdFrames[0].SimTimeSec);

            double sumAltSq = 0.0;
            double sumVelSq = 0.0;
            double sumRollSq = 0.0;
            double sumPitchSq = 0.0;

            for (int i = 0; i < n; i++)
            {
                var f = _fdFrames[i];
                double eAlt = System.Math.Abs(f.AltitudeAgl - f.TargetAltAgl);
                if (eAlt > m.AltitudeMaxErrorMeters) m.AltitudeMaxErrorMeters = eAlt;
                sumAltSq += eAlt * eAlt;

                double eVel = f.AirspeedMps - f.TargetAirspeedMps;
                sumVelSq += eVel * eVel;

                double eRoll = f.RollDeg - f.TargetRollDeg;
                sumRollSq += eRoll * eRoll;

                double ePitch = f.PitchDeg - f.TargetPitchDeg;
                sumPitchSq += ePitch * ePitch;

                if (f.AltitudeAgl > m.MaxAltitudeAglMeters) m.MaxAltitudeAglMeters = f.AltitudeAgl;
                if (f.AirspeedMps > m.MaxAirspeedMps) m.MaxAirspeedMps = f.AirspeedMps;
                double absRoll = System.Math.Abs(f.RollDeg);
                if (absRoll > m.MaxRollAbsDeg) m.MaxRollAbsDeg = absRoll;
            }

            m.AltitudeRmseMeters = System.Math.Sqrt(sumAltSq / n);
            m.AirspeedRmseMps = System.Math.Sqrt(sumVelSq / n);
            m.RollRmseDeg = System.Math.Sqrt(sumRollSq / n);
            m.PitchRmseDeg = System.Math.Sqrt(sumPitchSq / n);
            return m;
        }

        public string ExportToCsvString()
        {
            var sb = new StringBuilder(1024 + _fdFrames.Count * 160);
            var inv = CultureInfo.InvariantCulture;

            sb.AppendLine(
                "SimTimeSec,Category,Fidelity,IsArmed,IsGrounded," +
                "PosN_m,PosE_m,PosD_m,AltitudeAgl_m,TargetPosN_m,TargetPosE_m,TargetAltAgl_m," +
                "VelN_mps,VelE_mps,VelD_mps,Airspeed_mps,TargetAirspeed_mps,Alpha_deg,Beta_deg," +
                "Roll_deg,Pitch_deg,Yaw_deg,TargetRoll_deg,TargetPitch_deg,TargetYaw_deg," +
                "P_degps,Q_degps,R_degps,Fx_N,Fy_N,Fz_N,Mx_Nm,My_Nm,Mz_Nm," +
                "Act0,Act1,Act2,Act3,Act4,Act5,Act6,Act7,Act8,TotalThrust_N,Battery_V,Battery_A");

            for (int i = 0; i < _fdFrames.Count; i++)
            {
                var f = _fdFrames[i];
                sb.Append(f.SimTimeSec.ToString("F4", inv)).Append(',');
                sb.Append(f.Category).Append(',');
                sb.Append(f.Fidelity).Append(',');
                sb.Append(f.IsArmed ? 1 : 0).Append(',');
                sb.Append(f.IsGrounded ? 1 : 0).Append(',');
                sb.Append(f.PosN.ToString("F4", inv)).Append(',');
                sb.Append(f.PosE.ToString("F4", inv)).Append(',');
                sb.Append(f.PosD.ToString("F4", inv)).Append(',');
                sb.Append(f.AltitudeAgl.ToString("F4", inv)).Append(',');
                sb.Append(f.TargetPosN.ToString("F4", inv)).Append(',');
                sb.Append(f.TargetPosE.ToString("F4", inv)).Append(',');
                sb.Append(f.TargetAltAgl.ToString("F4", inv)).Append(',');
                sb.Append(f.VelN.ToString("F4", inv)).Append(',');
                sb.Append(f.VelE.ToString("F4", inv)).Append(',');
                sb.Append(f.VelD.ToString("F4", inv)).Append(',');
                sb.Append(f.AirspeedMps.ToString("F4", inv)).Append(',');
                sb.Append(f.TargetAirspeedMps.ToString("F4", inv)).Append(',');
                sb.Append(f.AlphaDeg.ToString("F3", inv)).Append(',');
                sb.Append(f.BetaDeg.ToString("F3", inv)).Append(',');
                sb.Append(f.RollDeg.ToString("F3", inv)).Append(',');
                sb.Append(f.PitchDeg.ToString("F3", inv)).Append(',');
                sb.Append(f.YawDeg.ToString("F3", inv)).Append(',');
                sb.Append(f.TargetRollDeg.ToString("F3", inv)).Append(',');
                sb.Append(f.TargetPitchDeg.ToString("F3", inv)).Append(',');
                sb.Append(f.TargetYawDeg.ToString("F3", inv)).Append(',');
                sb.Append(f.P_Degps.ToString("F3", inv)).Append(',');
                sb.Append(f.Q_Degps.ToString("F3", inv)).Append(',');
                sb.Append(f.R_Degps.ToString("F3", inv)).Append(',');
                sb.Append(f.FxFrd.ToString("F3", inv)).Append(',');
                sb.Append(f.FyFrd.ToString("F3", inv)).Append(',');
                sb.Append(f.FzFrd.ToString("F3", inv)).Append(',');
                sb.Append(f.MxNm.ToString("F4", inv)).Append(',');
                sb.Append(f.MyNm.ToString("F4", inv)).Append(',');
                sb.Append(f.MzNm.ToString("F4", inv)).Append(',');
                sb.Append(f.Act0.ToString("F4", inv)).Append(',');
                sb.Append(f.Act1.ToString("F4", inv)).Append(',');
                sb.Append(f.Act2.ToString("F4", inv)).Append(',');
                sb.Append(f.Act3.ToString("F4", inv)).Append(',');
                sb.Append(f.Act4.ToString("F4", inv)).Append(',');
                sb.Append(f.Act5.ToString("F4", inv)).Append(',');
                sb.Append(f.Act6.ToString("F4", inv)).Append(',');
                sb.Append(f.Act7.ToString("F4", inv)).Append(',');
                sb.Append(f.Act8.ToString("F4", inv)).Append(',');
                sb.Append(f.TotalThrustNewtons.ToString("F3", inv)).Append(',');
                sb.Append(f.BatteryVoltage.ToString("F3", inv)).Append(',');
                sb.AppendLine(f.BatteryCurrent.ToString("F3", inv));
            }

            return sb.ToString();
        }

        public void ExportToCsvFile(string filePath)
        {
            string? dir = Path.GetDirectoryName(filePath);
            if (!string.IsNullOrEmpty(dir))
                Directory.CreateDirectory(dir);
            File.WriteAllText(filePath, ExportToCsvString(), Encoding.UTF8);
        }

        public int LoadFromCsvFile(string filePath)
        {
            if (string.IsNullOrEmpty(filePath) || !File.Exists(filePath))
                return 0;
            return LoadFromCsvString(File.ReadAllText(filePath, Encoding.UTF8));
        }

        public int LoadFromCsvString(string csvContent)
        {
            Clear();
            if (string.IsNullOrWhiteSpace(csvContent))
                return 0;

            var inv = CultureInfo.InvariantCulture;
            using var reader = new StringReader(csvContent);
            string? header = reader.ReadLine();
            if (string.IsNullOrWhiteSpace(header))
                return 0;

            string? line;
            while ((line = reader.ReadLine()) != null)
            {
                if (string.IsNullOrWhiteSpace(line)) continue;
                string[] cols = line.Split(',');
                if (cols.Length < 46) continue;

                static double ParseD(string s, CultureInfo c) =>
                    double.TryParse(s, NumberStyles.Float | NumberStyles.AllowThousands, c, out double v) ? v : 0.0;

                Enum.TryParse<VehicleAirframeCategory>(cols[1].Trim(), true, out var cat);
                Enum.TryParse<SimulationFidelityLevel>(cols[2].Trim(), true, out var fid);

                var frame = new TelemetryLogFrame
                {
                    SimTimeSec = ParseD(cols[0], inv),
                    Category = cat,
                    Fidelity = fid,
                    IsArmed = cols[3].Trim() == "1" || cols[3].Trim().Equals("true", StringComparison.OrdinalIgnoreCase),
                    IsGrounded = cols[4].Trim() == "1" || cols[4].Trim().Equals("true", StringComparison.OrdinalIgnoreCase),
                    PosN = ParseD(cols[5], inv),
                    PosE = ParseD(cols[6], inv),
                    PosD = ParseD(cols[7], inv),
                    AltitudeAgl = ParseD(cols[8], inv),
                    TargetPosN = ParseD(cols[9], inv),
                    TargetPosE = ParseD(cols[10], inv),
                    TargetAltAgl = ParseD(cols[11], inv),
                    VelN = ParseD(cols[12], inv),
                    VelE = ParseD(cols[13], inv),
                    VelD = ParseD(cols[14], inv),
                    AirspeedMps = ParseD(cols[15], inv),
                    TargetAirspeedMps = ParseD(cols[16], inv),
                    AlphaDeg = ParseD(cols[17], inv),
                    BetaDeg = ParseD(cols[18], inv),
                    RollDeg = ParseD(cols[19], inv),
                    PitchDeg = ParseD(cols[20], inv),
                    YawDeg = ParseD(cols[21], inv),
                    TargetRollDeg = ParseD(cols[22], inv),
                    TargetPitchDeg = ParseD(cols[23], inv),
                    TargetYawDeg = ParseD(cols[24], inv),
                    P_Degps = ParseD(cols[25], inv),
                    Q_Degps = ParseD(cols[26], inv),
                    R_Degps = ParseD(cols[27], inv),
                    FxFrd = ParseD(cols[28], inv),
                    FyFrd = ParseD(cols[29], inv),
                    FzFrd = ParseD(cols[30], inv),
                    MxNm = ParseD(cols[31], inv),
                    MyNm = ParseD(cols[32], inv),
                    MzNm = ParseD(cols[33], inv),
                    Act0 = ParseD(cols[34], inv),
                    Act1 = ParseD(cols[35], inv),
                    Act2 = ParseD(cols[36], inv),
                    Act3 = ParseD(cols[37], inv),
                    Act4 = ParseD(cols[38], inv),
                    Act5 = ParseD(cols[39], inv),
                    Act6 = ParseD(cols[40], inv),
                    Act7 = ParseD(cols[41], inv),
                    Act8 = ParseD(cols[42], inv),
                    TotalThrustNewtons = ParseD(cols[43], inv),
                    BatteryVoltage = ParseD(cols[44], inv),
                    BatteryCurrent = ParseD(cols[45], inv)
                };
                RecordFrame(frame);
            }

            return _fdFrames.Count;
        }

        public string ExportToAcademicHtmlReportString(string title = "FlightDynamics 6-DOF Academic Flight Test Report")
        {
            var inv = CultureInfo.InvariantCulture;
            var metrics = ComputeMetrics();
            var sb = new StringBuilder(8192);

            sb.AppendLine("<!DOCTYPE html>");
            sb.AppendLine("<html lang=\"en\">");
            sb.AppendLine("<head>");
            sb.AppendLine("<meta charset=\"utf-8\"/>");
            sb.AppendLine($"<title>{InternalEscapeHtml(title)}</title>");
            sb.AppendLine("<style>");
            sb.AppendLine("body{font-family:-apple-system,BlinkMacSystemFont,'Segoe UI',Roboto,sans-serif;background:#0f141a;color:#e6edf3;margin:24px;}");
            sb.AppendLine("h1{margin:0 0 6px 0;font-size:22px;color:#58a6ff;} .sub{color:#8b949e;margin-bottom:18px;font-size:13px;}");
            sb.AppendLine(".grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(170px,1fr));gap:10px;margin-bottom:20px;}");
            sb.AppendLine(".card{background:#161b22;border:1px solid #30363d;border-radius:8px;padding:12px;}");
            sb.AppendLine(".card .k{font-size:11px;color:#8b949e;text-transform:uppercase;} .card .v{font-size:19px;font-weight:700;margin-top:4px;color:#79c0ff;}");
            sb.AppendLine(".chart-box{background:#161b22;border:1px solid #30363d;border-radius:8px;padding:14px;margin-bottom:16px;}");
            sb.AppendLine(".chart-title{font-size:14px;font-weight:600;margin-bottom:8px;color:#d2a8ff;}");
            sb.AppendLine("</style>");
            sb.AppendLine("</head><body>");
            sb.AppendLine($"<h1>{InternalEscapeHtml(title)}</h1>");
            sb.AppendLine($"<div class=\"sub\">Generated UTC: {DateTime.UtcNow.ToString("yyyy-MM-dd HH:mm:ss", inv)} | Samples: {metrics.SampleCount} | Duration: {metrics.DurationSec.ToString("F2", inv)} s</div>");

            sb.AppendLine("<div class=\"grid\">");
            InternalAppendMetricCard(sb, "Altitude RMSE", metrics.AltitudeRmseMeters.ToString("F3", inv) + " m");
            InternalAppendMetricCard(sb, "Max Alt Error", metrics.AltitudeMaxErrorMeters.ToString("F3", inv) + " m");
            InternalAppendMetricCard(sb, "Airspeed RMSE", metrics.AirspeedRmseMps.ToString("F3", inv) + " m/s");
            InternalAppendMetricCard(sb, "Roll RMSE", metrics.RollRmseDeg.ToString("F2", inv) + " deg");
            InternalAppendMetricCard(sb, "Pitch RMSE", metrics.PitchRmseDeg.ToString("F2", inv) + " deg");
            InternalAppendMetricCard(sb, "Peak Altitude", metrics.MaxAltitudeAglMeters.ToString("F2", inv) + " m");
            InternalAppendMetricCard(sb, "Peak Airspeed", metrics.MaxAirspeedMps.ToString("F2", inv) + " m/s");
            InternalAppendMetricCard(sb, "Peak |Roll|", metrics.MaxRollAbsDeg.ToString("F2", inv) + " deg");
            sb.AppendLine("</div>");

            if (_fdFrames.Count >= 2)
            {
                InternalBuildSvgChart(sb, "1. Altitude & Vertical Speed Tracking [m, m/s]",
                    new[] { "Altitude AGL [m]", "Target Alt [m]", "Vertical Vel -VelD [m/s]" },
                    new[] { "#3fb950", "#8b949e", "#58a6ff" },
                    f => f.AltitudeAgl, f => f.TargetAltAgl, f => -f.VelD);

                InternalBuildSvgChart(sb, "2. Attitude Tracking (Roll, Pitch, Target Pitch) [deg]",
                    new[] { "Roll phi [deg]", "Pitch theta [deg]", "Target Pitch [deg]", "Yaw psi [deg]" },
                    new[] { "#f85149", "#58a6ff", "#d29922", "#3fb950" },
                    f => f.RollDeg, f => f.PitchDeg, f => f.TargetPitchDeg, f => f.YawDeg);

                InternalBuildSvgChart(sb, "3. Aerodynamics: Airspeed, AoA & Sideslip [m/s, deg]",
                    new[] { "Airspeed V [m/s]", "Target V [m/s]", "Alpha AoA [deg]", "Beta [deg]" },
                    new[] { "#79c0ff", "#8b949e", "#d2a8ff", "#ffa657" },
                    f => f.AirspeedMps, f => f.TargetAirspeedMps, f => f.AlphaDeg, f => f.BetaDeg);

                InternalBuildSvgChart(sb, "4. Body Angular Rates & Aero Moments [deg/s, Nm]",
                    new[] { "p [deg/s]", "q [deg/s]", "r [deg/s]", "Mx [Nm]", "My [Nm]" },
                    new[] { "#ff7b72", "#79c0ff", "#7ee787", "#d2a8ff", "#ffa657" },
                    f => f.P_Degps, f => f.Q_Degps, f => f.R_Degps, f => f.MxNm, f => f.MyNm);

                InternalBuildSvgChart(sb, "5. Normalized Actuator Commands (Throttle, Aileron, Elevator, Rudder)",
                    new[] { "Ch0/Ch4 Thr", "Ch5 Ail-L", "Ch7 Elev", "Ch8 Rud" },
                    new[] { "#3fb950", "#f85149", "#58a6ff", "#d2a8ff" },
                    f => System.Math.Max(f.Act0, f.Act4), f => f.Act5, f => f.Act7, f => f.Act8);
            }
            else
            {
                sb.AppendLine("<div class=\"card\">No flight frames recorded.</div>");
            }

            sb.AppendLine("</body></html>");
            return sb.ToString();
        }

        public void ExportToAcademicHtmlReportFile(string filePath, string title = "FlightDynamics 6-DOF Academic Flight Test Report")
        {
            string? dir = Path.GetDirectoryName(filePath);
            if (!string.IsNullOrEmpty(dir))
                Directory.CreateDirectory(dir);
            File.WriteAllText(filePath, ExportToAcademicHtmlReportString(title), Encoding.UTF8);
        }

        private static void InternalAppendMetricCard(StringBuilder sb, string label, string value)
        {
            sb.AppendLine($"<div class=\"card\"><div class=\"k\">{InternalEscapeHtml(label)}</div><div class=\"v\">{InternalEscapeHtml(value)}</div></div>");
        }

        private void InternalBuildSvgChart(
            StringBuilder sb,
            string title,
            string[] seriesLabels,
            string[] colors,
            params Func<TelemetryLogFrame, double>[] extractors)
        {
            const double w = 960.0;
            const double h = 210.0;
            const double padL = 54.0;
            const double padR = 16.0;
            const double padT = 14.0;
            const double padB = 26.0;
            double plotW = w - padL - padR;
            double plotH = h - padT - padB;
            var inv = CultureInfo.InvariantCulture;

            int n = _fdFrames.Count;
            double t0 = _fdFrames[0].SimTimeSec;
            double t1 = _fdFrames[n - 1].SimTimeSec;
            double dt = System.Math.Max(1e-3, t1 - t0);

            double minY = double.PositiveInfinity;
            double maxY = double.NegativeInfinity;
            int step = System.Math.Max(1, n / 480);

            for (int i = 0; i < n; i += step)
            {
                var f = _fdFrames[i];
                for (int s = 0; s < extractors.Length; s++)
                {
                    double val = extractors[s](f);
                    if (!double.IsNaN(val) && !double.IsInfinity(val))
                    {
                        if (val < minY) minY = val;
                        if (val > maxY) maxY = val;
                    }
                }
            }

            if (double.IsInfinity(minY) || double.IsInfinity(maxY))
            {
                minY = -1.0;
                maxY = 1.0;
            }
            if (System.Math.Abs(maxY - minY) < 1e-3)
            {
                minY -= 0.5;
                maxY += 0.5;
            }
            else
            {
                double margin = 0.08 * (maxY - minY);
                minY -= margin;
                maxY += margin;
            }

            sb.AppendLine("<div class=\"chart-box\">");
            sb.AppendLine($"<div class=\"chart-title\">{InternalEscapeHtml(title)}</div>");
            sb.Append("<div style=\"font-size:11px;margin-bottom:6px;\">");
            for (int s = 0; s < seriesLabels.Length && s < colors.Length; s++)
            {
                sb.Append($"<span style=\"color:{colors[s]};margin-right:14px;\">&#9632; {InternalEscapeHtml(seriesLabels[s])}</span>");
            }
            sb.AppendLine("</div>");

            sb.AppendLine($"<svg viewBox=\"0 0 {w} {h}\" width=\"100%\" height=\"210\" xmlns=\"http://www.w3.org/2000/svg\" style=\"background:#0d1117;border-radius:4px;\">");

            // Horizontal gridlines & Y labels
            for (int g = 0; g <= 4; g++)
            {
                double frac = g / 4.0;
                double yPx = padT + frac * plotH;
                double val = maxY - frac * (maxY - minY);
                sb.AppendLine($"<line x1=\"{padL.ToString("F1", inv)}\" y1=\"{yPx.ToString("F1", inv)}\" x2=\"{(w - padR).ToString("F1", inv)}\" y2=\"{yPx.ToString("F1", inv)}\" stroke=\"#21262d\" stroke-width=\"1\"/>");
                sb.AppendLine($"<text x=\"4\" y=\"{(yPx + 4).ToString("F1", inv)}\" fill=\"#8b949e\" font-size=\"10\">{val.ToString("F2", inv)}</text>");
            }

            // Polylines
            for (int s = 0; s < extractors.Length; s++)
            {
                var pts = new StringBuilder();
                for (int i = 0; i < n; i += step)
                {
                    var f = _fdFrames[i];
                    double xPx = padL + ((f.SimTimeSec - t0) / dt) * plotW;
                    double v = System.Math.Clamp(extractors[s](f), minY, maxY);
                    double yPx = padT + (1.0 - (v - minY) / (maxY - minY)) * plotH;
                    pts.Append(xPx.ToString("F1", inv)).Append(',').Append(yPx.ToString("F1", inv)).Append(' ');
                }
                string col = colors[s % colors.Length];
                sb.AppendLine($"<polyline fill=\"none\" stroke=\"{col}\" stroke-width=\"1.8\" points=\"{pts}\"/>");
            }

            sb.AppendLine($"<text x=\"{padL.ToString("F1", inv)}\" y=\"{(h - 6).ToString("F1", inv)}\" fill=\"#8b949e\" font-size=\"10\">t={t0.ToString("F1", inv)}s</text>");
            sb.AppendLine($"<text x=\"{(w - 60).ToString("F1", inv)}\" y=\"{(h - 6).ToString("F1", inv)}\" fill=\"#8b949e\" font-size=\"10\">t={t1.ToString("F1", inv)}s</text>");
            sb.AppendLine("</svg></div>");
        }

        private static string InternalEscapeHtml(string s)
        {
            if (string.IsNullOrEmpty(s)) return "";
            return s.Replace("&", "&amp;").Replace("<", "&lt;").Replace(">", "&gt;").Replace("\"", "&quot;");
        }
    }
}
