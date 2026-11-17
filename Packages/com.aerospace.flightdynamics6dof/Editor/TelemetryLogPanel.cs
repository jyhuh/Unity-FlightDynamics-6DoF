// Licensed under the Apache License, Version 2.0.
#nullable enable
using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using FlightDynamics6DoF.Core.Aerodynamics;
using FlightDynamics6DoF.Core.Dynamics;
using FlightDynamics6DoF.Core.Gnc;
using FlightDynamics6DoF.Core.Math;
using FlightDynamics6DoF.Core.Specs;
using FlightDynamics6DoF.Unity;
using UnityEditor;
using UnityEngine;
using Debug = UnityEngine.Debug;

namespace FlightDynamics6DoF.Editor
{
    /// <summary>
    /// Step 5: Live flight telemetry plotting, offline fixed-wing demo log, and Assets/FlightLogs browser.
    /// </summary>
    internal sealed class TelemetryLogPanel
    {
        private float _plotWindowSec = 30.0f;
        private double _lastPlotRepaintTime;
        private readonly FlightTelemetryLogger _offlinePreviewLogger = new FlightTelemetryLogger();
        private readonly FlightTelemetryLogger _loadedDiskLogger = new FlightTelemetryLogger();
        private string[] _savedLogPaths = Array.Empty<string>();
        private string[] _savedLogLabels = Array.Empty<string>();
        private int _selectedSavedLogIndex = -1;
        private string _loadedDiskLogPath = "";
        private bool _diskLogListScanned = false;

        public bool ShouldRepaintOnPlay(double timeSinceStartup)
        {
            if (timeSinceStartup - _lastPlotRepaintTime < 0.08)
                return false;
            _lastPlotRepaintTime = timeSinceStartup;
            return true;
        }

        public FlightTelemetryLogger GetActiveLogger(FlightDynamicsVehicle? sceneVehicle)
        {
            if (sceneVehicle != null && sceneVehicle.TelemetryLogger.Frames.Count > 0)
                return sceneVehicle.TelemetryLogger;
            if (_offlinePreviewLogger.Frames.Count > 0)
                return _offlinePreviewLogger;
            if (!_diskLogListScanned)
                RefreshSavedFlightLogList(autoLoadLatestIfEmpty: true);
            if (_loadedDiskLogger.Frames.Count > 0)
                return _loadedDiskLogger;
            return sceneVehicle != null ? sceneVehicle.TelemetryLogger : _offlinePreviewLogger;
        }

        public void RefreshSavedFlightLogList(bool autoLoadLatestIfEmpty)
        {
            _diskLogListScanned = true;
            string logDir = FlightLogRetentionSettings.GetDefaultLogDirectory();
            if (!Directory.Exists(logDir))
            {
                _savedLogPaths = Array.Empty<string>();
                _savedLogLabels = Array.Empty<string>();
                _selectedSavedLogIndex = -1;
                return;
            }

            var files = new DirectoryInfo(logDir).GetFiles("*.csv", SearchOption.TopDirectoryOnly)
                .OrderByDescending(f => f.LastWriteTimeUtc)
                .ToArray();

            _savedLogPaths = new string[files.Length];
            _savedLogLabels = new string[files.Length];
            for (int i = 0; i < files.Length; i++)
            {
                _savedLogPaths[i] = files[i].FullName;
                double kb = files[i].Length / 1024.0;
                _savedLogLabels[i] = $"{files[i].Name} ({kb:F1} KB)";
            }

            if (files.Length > 0)
            {
                int prevIdx = Array.IndexOf(_savedLogPaths, _loadedDiskLogPath);
                _selectedSavedLogIndex = prevIdx >= 0 ? prevIdx : 0;
                if (autoLoadLatestIfEmpty && _loadedDiskLogger.Frames.Count == 0)
                    LoadDiskLogAtIndex(_selectedSavedLogIndex);
            }
            else
            {
                _selectedSavedLogIndex = -1;
            }
        }

        public void Draw(FlightDynamicsEngineeringWizard host)
        {
            if (!_diskLogListScanned)
                RefreshSavedFlightLogList(autoLoadLatestIfEmpty: true);

            var vehicle = host.SceneVehicle;
            var logger = GetActiveLogger(vehicle);
            var metrics = logger.ComputeMetrics();
            string logDir = FlightLogRetentionSettings.GetDefaultLogDirectory();
            var (csvCount, _, totalMb) = FlightLogRetentionSettings.MeasureDirectoryUsage(logDir);
            var retention = FlightLogRetentionSettings.Load();

            EditorGUILayout.BeginVertical(EditorStyles.helpBox);
            EditorGUILayout.BeginHorizontal();
            GUILayout.Label("Step 5.1: Real-Time Telemetry & Auto-Logging (Assets/FlightLogs/)", EditorStyles.boldLabel);
            GUILayout.FlexibleSpace();
            GUI.backgroundColor = new Color(0.85f, 0.92f, 1f);
            if (GUILayout.Button("⚙ Log Retention Settings", GUILayout.Width(165), GUILayout.Height(22)))
                FlightLogSettingsWindow.ShowWindow();
            GUI.backgroundColor = Color.white;
            EditorGUILayout.EndHorizontal();

            EditorGUILayout.BeginHorizontal(EditorStyles.helpBox);
            GUILayout.Label($"Saved Logs ({csvCount}/{retention.MaxLogFilesOnDisk}, {totalMb:F1}/{retention.MaxTotalLogSizeMb:F0} MB):", GUILayout.Width(240));
            if (_savedLogLabels.Length > 0)
            {
                int nextIdx = EditorGUILayout.Popup(Mathf.Clamp(_selectedSavedLogIndex, 0, _savedLogLabels.Length - 1), _savedLogLabels);
                if (nextIdx != _selectedSavedLogIndex)
                {
                    _selectedSavedLogIndex = nextIdx;
                    LoadDiskLogAtIndex(_selectedSavedLogIndex);
                }
                if (GUILayout.Button("Reload Selected", GUILayout.Width(115)))
                    LoadDiskLogAtIndex(_selectedSavedLogIndex);
            }
            else
            {
                GUILayout.Label("No CSV logs found in Assets/FlightLogs/", EditorStyles.miniLabel);
            }
            if (GUILayout.Button("Refresh", GUILayout.Width(75)))
                RefreshSavedFlightLogList(autoLoadLatestIfEmpty: true);
            EditorGUILayout.EndHorizontal();

            EditorGUILayout.BeginHorizontal();
            GUI.backgroundColor = logger.IsRecording ? new Color(1f, 0.6f, 0.6f) : new Color(0.55f, 1f, 0.65f);
            if (GUILayout.Button(logger.IsRecording ? "Pause Recording" : "Start/Resume Recording", GUILayout.Height(26)))
            {
                if (logger.IsRecording) logger.PauseRecording();
                else logger.StartRecording();
            }
            GUI.backgroundColor = Color.white;

            if (GUILayout.Button("Clear Buffer", GUILayout.Height(26)))
            {
                logger.Clear();
                _loadedDiskLogger.Clear();
                _loadedDiskLogPath = "";
            }

            if (GUILayout.Button("Generate 15s FW Demo", GUILayout.Height(26)))
                GenerateClosedLoopFixedWingDemoLog();

            if (GUILayout.Button("Save to Assets/FlightLogs", GUILayout.Height(26)))
            {
                Directory.CreateDirectory(logDir);
                string stamp = DateTime.Now.ToString("yyyyMMdd_HHmmss");
                string csvPath = Path.Combine(logDir, $"manual_{stamp}.csv");
                string htmlPath = Path.Combine(logDir, $"manual_report_{stamp}.html");
                logger.ExportToCsvFile(csvPath);
                logger.ExportToAcademicHtmlReportFile(htmlPath, $"FlightDynamics 6-DOF Report - {host.GetCurrentConfig().AircraftName}");
                retention.ApplyRetentionPolicy(logDir);
                AssetDatabase.Refresh();
                RefreshSavedFlightLogList(autoLoadLatestIfEmpty: false);
                Debug.Log($"[FlightDynamics6DoF] Saved logs to Assets/FlightLogs/: {Path.GetFileName(csvPath)}, {Path.GetFileName(htmlPath)}");
            }

            if (GUILayout.Button("Open Log Folder", GUILayout.Height(26)))
            {
                Directory.CreateDirectory(logDir);
                EditorUtility.RevealInFinder(logDir);
            }
            EditorGUILayout.EndHorizontal();

            EditorGUILayout.BeginHorizontal();
            logger.SampleRateHz = EditorGUILayout.Slider("Sample Rate [Hz]", (float)logger.SampleRateHz, 10f, 250f);
            _plotWindowSec = EditorGUILayout.Slider("Plot Window [s]", _plotWindowSec, 5f, 120f);
            EditorGUILayout.EndHorizontal();

            string sourceLabel = (vehicle != null && vehicle.TelemetryLogger.Frames.Count > 0)
                ? "Live Simulation Buffer"
                : (!string.IsNullOrEmpty(_loadedDiskLogPath) ? $"Disk Log: {Path.GetFileName(_loadedDiskLogPath)}" : "Offline Demo / Empty Buffer");

            EditorGUILayout.HelpBox(
                $"[{sourceLabel}] Frames: {metrics.SampleCount} (Cap {retention.MaxFramesInMemory:N0}) | Duration: {metrics.DurationSec:F2} s | " +
                $"Alt RMSE: {metrics.AltitudeRmseMeters:F3} m (Max Err {metrics.AltitudeMaxErrorMeters:F2} m) | " +
                $"Airspeed RMSE: {metrics.AirspeedRmseMps:F2} m/s | Roll RMSE: {metrics.RollRmseDeg:F2} deg | Pitch RMSE: {metrics.PitchRmseDeg:F2} deg | Max|Roll|: {metrics.MaxRollAbsDeg:F2} deg",
                MessageType.Info);
            EditorGUILayout.EndVertical();

            EditorGUILayout.Space(6);

            var frames = logger.Frames;
            if (frames.Count < 2)
            {
                EditorGUILayout.HelpBox("No flight telemetry recorded yet. Recording starts automatically on Play and saves to Assets/FlightLogs/ on Stop.", MessageType.Warning);
                return;
            }

            DrawVectorPlotChart(host, "1. Altitude & Climb Rate Tracking [m, m/s]", frames,
                new[] { "Altitude AGL [m]", "Target Alt [m]", "Climb -VelD [m/s]" },
                new[] { new Color(0.3f, 0.9f, 0.4f), Color.gray, new Color(0.35f, 0.7f, 1f) },
                f => f.AltitudeAgl, f => f.TargetAltAgl, f => -f.VelD);

            DrawVectorPlotChart(host, "2. Attitude Tracking (Roll / Pitch / Yaw) [deg]", frames,
                new[] { "Roll phi", "Pitch theta", "Target Pitch", "Yaw psi" },
                new[] { new Color(1f, 0.35f, 0.35f), new Color(0.35f, 0.7f, 1f), new Color(0.95f, 0.75f, 0.25f), new Color(0.3f, 0.9f, 0.45f) },
                f => f.RollDeg, f => f.PitchDeg, f => f.TargetPitchDeg, f => f.YawDeg);

            DrawVectorPlotChart(host, "3. Aerodynamics (Airspeed / AoA / Sideslip) [m/s, deg]", frames,
                new[] { "Airspeed V [m/s]", "Target V [m/s]", "Alpha [deg]", "Beta [deg]" },
                new[] { new Color(0.45f, 0.8f, 1f), Color.gray, new Color(0.85f, 0.55f, 1f), new Color(1f, 0.65f, 0.3f) },
                f => f.AirspeedMps, f => f.TargetAirspeedMps, f => f.AlphaDeg, f => f.BetaDeg);

            DrawVectorPlotChart(host, "4. Body Rates & Aerodynamic Moments [deg/s, Nm]", frames,
                new[] { "p [deg/s]", "q [deg/s]", "r [deg/s]", "Mx [Nm]", "My [Nm]" },
                new[] { new Color(1f, 0.45f, 0.45f), new Color(0.45f, 0.8f, 1f), new Color(0.45f, 0.95f, 0.55f), new Color(0.85f, 0.55f, 1f), new Color(1f, 0.65f, 0.3f) },
                f => f.P_Degps, f => f.Q_Degps, f => f.R_Degps, f => f.MxNm, f => f.MyNm);

            DrawVectorPlotChart(host, "5. Normalized Actuator Outputs [-1..1]", frames,
                new[] { "Ch0/4 Thr", "Ch5 Ail-L", "Ch7 Elev", "Ch8 Rud" },
                new[] { new Color(0.3f, 0.9f, 0.45f), new Color(1f, 0.35f, 0.35f), new Color(0.35f, 0.7f, 1f), new Color(0.85f, 0.55f, 1f) },
                f => System.Math.Max(f.Act0, f.Act4), f => f.Act5, f => f.Act7, f => f.Act8);
        }

        private void DrawVectorPlotChart(
            EditorWindow host,
            string title,
            IReadOnlyList<TelemetryLogFrame> frames,
            string[] labels,
            Color[] colors,
            params Func<TelemetryLogFrame, double>[] extractors)
        {
            EditorGUILayout.BeginVertical(EditorStyles.helpBox);
            EditorGUILayout.BeginHorizontal();
            GUILayout.Label(title, EditorStyles.boldLabel);
            GUILayout.FlexibleSpace();
            for (int s = 0; s < labels.Length && s < colors.Length; s++)
            {
                var prev = GUI.contentColor;
                GUI.contentColor = colors[s];
                GUILayout.Label($"■ {labels[s]}", EditorStyles.miniBoldLabel);
                GUI.contentColor = prev;
            }
            EditorGUILayout.EndHorizontal();

            Rect plotRect = GUILayoutUtility.GetRect(100, 130, GUILayout.ExpandWidth(true));
            if (Event.current.type == EventType.Repaint || plotRect.Contains(Event.current.mousePosition))
            {
                EditorGUI.DrawRect(plotRect, new Color(0.08f, 0.10f, 0.13f, 1f));

                int n = frames.Count;
                double tMax = frames[n - 1].SimTimeSec;
                double tMin = System.Math.Max(frames[0].SimTimeSec, tMax - _plotWindowSec);
                int startIdx = 0;
                for (int i = n - 1; i >= 0; i--)
                {
                    if (frames[i].SimTimeSec <= tMin) { startIdx = i; break; }
                }

                double dt = System.Math.Max(1e-3, tMax - tMin);
                double minY = double.PositiveInfinity;
                double maxY = double.NegativeInfinity;
                int stride = System.Math.Max(1, (n - startIdx) / 280);

                for (int i = startIdx; i < n; i += stride)
                {
                    var f = frames[i];
                    for (int s = 0; s < extractors.Length; s++)
                    {
                        double v = extractors[s](f);
                        if (!double.IsNaN(v) && !double.IsInfinity(v))
                        {
                            if (v < minY) minY = v;
                            if (v > maxY) maxY = v;
                        }
                    }
                }

                if (double.IsInfinity(minY) || double.IsInfinity(maxY)) { minY = -1; maxY = 1; }
                if (System.Math.Abs(maxY - minY) < 1e-3) { minY -= 0.5; maxY += 0.5; }
                else { double pad = 0.08 * (maxY - minY); minY -= pad; maxY += pad; }

                Color gridCol = new Color(1f, 1f, 1f, 0.08f);
                for (int g = 1; g <= 3; g++)
                {
                    float yG = plotRect.y + (g / 4f) * plotRect.height;
                    EditorGUI.DrawRect(new Rect(plotRect.x, yG, plotRect.width, 1f), gridCol);
                }

                for (int s = 0; s < extractors.Length; s++)
                {
                    var pts = new List<Vector3>();
                    for (int i = startIdx; i < n; i += stride)
                    {
                        var f = frames[i];
                        float x = plotRect.x + (float)((f.SimTimeSec - tMin) / dt) * plotRect.width;
                        double clamped = System.Math.Clamp(extractors[s](f), minY, maxY);
                        float y = plotRect.yMax - (float)((clamped - minY) / (maxY - minY)) * plotRect.height;
                        pts.Add(new Vector3(x, y, 0f));
                    }
                    if (pts.Count >= 2)
                    {
                        Handles.color = colors[s % colors.Length];
                        Handles.DrawAAPolyLine(2.0f, pts.ToArray());
                    }
                }

                GUI.Label(new Rect(plotRect.x + 4, plotRect.y + 2, 120, 16), $"{maxY:F2}", EditorStyles.miniLabel);
                GUI.Label(new Rect(plotRect.x + 4, plotRect.yMax - 16, 120, 16), $"{minY:F2}", EditorStyles.miniLabel);

                if (plotRect.Contains(Event.current.mousePosition))
                {
                    float mouseFrac = (Event.current.mousePosition.x - plotRect.x) / Mathf.Max(1f, plotRect.width);
                    double hoverTime = tMin + mouseFrac * dt;
                    int nearestIdx = startIdx;
                    double bestDiff = double.MaxValue;
                    for (int i = startIdx; i < n; i += stride)
                    {
                        double d = System.Math.Abs(frames[i].SimTimeSec - hoverTime);
                        if (d < bestDiff) { bestDiff = d; nearestIdx = i; }
                    }
                    EditorGUI.DrawRect(new Rect(Event.current.mousePosition.x, plotRect.y, 1f, plotRect.height), new Color(1f, 1f, 1f, 0.35f));
                    var hf = frames[nearestIdx];
                    string readout = $"t={hf.SimTimeSec:F2}s:";
                    for (int s = 0; s < extractors.Length && s < labels.Length; s++)
                        readout += $" {labels[s]}={extractors[s](hf):F2}";
                    GUI.Label(new Rect(plotRect.x + 80, plotRect.y + 2, plotRect.width - 84, 18), readout, EditorStyles.miniBoldLabel);
                    host.Repaint();
                }
            }
            EditorGUILayout.EndVertical();
        }

        private void LoadDiskLogAtIndex(int index)
        {
            if (index < 0 || index >= _savedLogPaths.Length) return;
            string path = _savedLogPaths[index];
            var retention = FlightLogRetentionSettings.Load();
            _loadedDiskLogger.MaxFrames = retention.MaxFramesInMemory;
            int count = _loadedDiskLogger.LoadFromCsvFile(path);
            if (count > 0)
                _loadedDiskLogPath = path;
        }

        private void GenerateClosedLoopFixedWingDemoLog()
        {
            var fw = AircraftConfigSpec.CreateFixedWingUav();
            var db = StabilityAndTrimAnalyzer.AnalyzeAndBake(fw);
            var table = AirframeControlParameterTable.CreateFixedWingDefault(fw, db);
            var engine = new MultiFidelitySixDofEngine(fw, db) { FidelityLevel = SimulationFidelityLevel.Level2_6DoFBladeAndStrip };
            var ctrl = new CascadedFlightController();
            ctrl.ApplyParameterTable(table);

            engine.ResetState(Vec3d.Zero, Quatd.Identity);
            ctrl.CommandTakeoff(Vec3d.Zero, 0.0, 25.0);

            _offlinePreviewLogger.Clear();
            _offlinePreviewLogger.StartRecording();
            _offlinePreviewLogger.SampleRateHz = 50.0;

            Span<double> u16 = stackalloc double[16];
            double dt = 0.005;
            for (int step = 0; step < 3000; step++)
            {
                ctrl.ComputeControlStep(dt, fw, db, engine.State, u16);
                engine.SetActuatorControls(u16);
                engine.StepSimulation(dt);
                _offlinePreviewLogger.RecordStep(engine, ctrl);
            }
        }
    }
}

