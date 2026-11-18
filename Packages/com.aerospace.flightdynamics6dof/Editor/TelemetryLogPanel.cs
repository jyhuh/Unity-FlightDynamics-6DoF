// Licensed under the Apache License, Version 2.0.
#nullable enable
using System.Collections.Generic;
using System.IO;
using FlightDynamics6DoF.Core.Dynamics;
using FlightDynamics6DoF.Unity;
using UnityEditor;
using UnityEngine;

namespace FlightDynamics6DoF.Editor
{
    /// <summary>
    /// Step 5: Coordinator panel for live flight telemetry plotting, time-axis navigation,
    /// Command-vs-Response comparison charts, and user-defined custom charts.
    /// </summary>
    internal sealed class TelemetryLogPanel
    {
        private readonly TelemetryLogRepository _repository = new TelemetryLogRepository();
        private readonly TelemetryTimeViewport _viewport = new TelemetryTimeViewport();
        private readonly List<TelemetryCustomChart> _customCharts = new List<TelemetryCustomChart>();
        private double _lastPlotRepaintTime;

        public bool ShouldRepaintOnPlay(double timeSinceStartup)
        {
            if (timeSinceStartup - _lastPlotRepaintTime < 0.08)
                return false;
            _lastPlotRepaintTime = timeSinceStartup;
            return true;
        }

        public FlightTelemetryLogger GetActiveLogger(FlightDynamicsVehicle? sceneVehicle)
            => _repository.GetActiveLogger(sceneVehicle);

        public void RefreshSavedFlightLogList(bool autoLoadLatestIfEmpty)
            => _repository.RefreshSavedFlightLogList(autoLoadLatestIfEmpty);

        public void Draw(FlightDynamicsEngineeringWizard host)
        {
            if (!_repository.DiskLogListScanned)
                _repository.RefreshSavedFlightLogList(autoLoadLatestIfEmpty: true);

            var vehicle = host.SceneVehicle;
            var logger = _repository.GetActiveLogger(vehicle);
            var metrics = logger.ComputeMetrics();
            string logDir = FlightLogRetentionSettings.GetDefaultLogDirectory();
            var (csvCount, _, totalMb) = FlightLogRetentionSettings.MeasureDirectoryUsage(logDir);
            var retention = FlightLogRetentionSettings.Load();

            EditorGUILayout.BeginVertical(EditorStyles.helpBox);
            DrawHeaderBar();
            DrawSavedLogsBar(csvCount, totalMb, retention);
            DrawActionButtons(host, logger, logDir, retention);
            DrawSamplingAndViewportControls(logger);
            DrawMetricsSummary(vehicle, metrics, retention);
            EditorGUILayout.EndVertical();

            EditorGUILayout.Space(6);

            var frames = logger.Frames;
            if (frames.Count < 2)
            {
                EditorGUILayout.HelpBox("No flight telemetry recorded yet. Recording starts automatically on Play and saves to Assets/FlightLogs/ on Stop.", MessageType.Warning);
                return;
            }

            foreach (var preset in TelemetryChannelRegistry.GetBuiltInPresets(frames[0].Category))
                TelemetryChartRenderer.Draw(host, preset, frames, _viewport);

            DrawCustomCharts(host, frames);
        }

        private static void DrawHeaderBar()
        {
            EditorGUILayout.BeginHorizontal();
            GUILayout.Label("Step 5.1: Real-Time Telemetry & Auto-Logging (Assets/FlightLogs/)", EditorStyles.boldLabel);
            GUILayout.FlexibleSpace();
            GUI.backgroundColor = new Color(0.85f, 0.92f, 1f);
            if (GUILayout.Button("⚙ Log Retention Settings", GUILayout.Width(165), GUILayout.Height(22)))
                FlightLogSettingsWindow.ShowWindow();
            GUI.backgroundColor = Color.white;
            EditorGUILayout.EndHorizontal();
        }

        private void DrawSavedLogsBar(int csvCount, double totalMb, FlightLogRetentionSettings retention)
        {
            EditorGUILayout.BeginHorizontal(EditorStyles.helpBox);
            GUILayout.Label($"Saved Logs ({csvCount}/{retention.MaxLogFilesOnDisk}, {totalMb:F1}/{retention.MaxTotalLogSizeMb:F0} MB):", GUILayout.Width(240));
            if (_repository.SavedLogLabels.Length > 0)
            {
                int nextIdx = EditorGUILayout.Popup(
                    Mathf.Clamp(_repository.SelectedSavedLogIndex, 0, _repository.SavedLogLabels.Length - 1),
                    _repository.SavedLogLabels);
                if (nextIdx != _repository.SelectedSavedLogIndex)
                {
                    _repository.SelectedSavedLogIndex = nextIdx;
                    _repository.LoadDiskLogAtIndex(nextIdx);
                }
                if (GUILayout.Button("Reload Selected", GUILayout.Width(115)))
                    _repository.LoadDiskLogAtIndex(_repository.SelectedSavedLogIndex);
            }
            else
            {
                GUILayout.Label("No CSV logs found in Assets/FlightLogs/", EditorStyles.miniLabel);
            }
            if (GUILayout.Button("Refresh", GUILayout.Width(75)))
                _repository.RefreshSavedFlightLogList(autoLoadLatestIfEmpty: true);
            EditorGUILayout.EndHorizontal();
        }

        private void DrawActionButtons(
            FlightDynamicsEngineeringWizard host,
            FlightTelemetryLogger logger,
            string logDir,
            FlightLogRetentionSettings retention)
        {
            EditorGUILayout.BeginHorizontal();
            GUI.backgroundColor = logger.IsRecording ? new Color(1f, 0.6f, 0.6f) : new Color(0.55f, 1f, 0.65f);
            if (GUILayout.Button(logger.IsRecording ? "Pause Recording" : "Start/Resume Recording", GUILayout.Height(26)))
            {
                if (logger.IsRecording) logger.PauseRecording();
                else logger.StartRecording();
            }
            GUI.backgroundColor = Color.white;

            if (GUILayout.Button("Clear Buffer", GUILayout.Height(26)))
                _repository.ClearBuffers(logger);

            if (GUILayout.Button("Generate 15s FW Demo", GUILayout.Height(26)))
                _repository.GenerateClosedLoopFixedWingDemoLog();

            if (GUILayout.Button("Save to Assets/FlightLogs", GUILayout.Height(26)))
                _repository.SaveManualSnapshot(logger, host.GetCurrentConfig().AircraftName, retention);

            if (GUILayout.Button("Open Log Folder", GUILayout.Height(26)))
            {
                Directory.CreateDirectory(logDir);
                EditorUtility.RevealInFinder(logDir);
            }
            EditorGUILayout.EndHorizontal();
        }

        private void DrawSamplingAndViewportControls(FlightTelemetryLogger logger)
        {
            EditorGUILayout.BeginHorizontal();
            logger.SampleRateHz = EditorGUILayout.Slider("Sample Rate [Hz]", (float)logger.SampleRateHz, 10f, 250f);
            float nextWin = EditorGUILayout.Slider(
                "Plot Window [s]",
                _viewport.WindowSec,
                TelemetryTimeViewport.MinWindowSec,
                TelemetryTimeViewport.MaxWindowSec);
            if (!Mathf.Approximately(nextWin, _viewport.WindowSec))
                _viewport.WindowSec = nextWin;
            EditorGUILayout.EndHorizontal();

            var frames = logger.Frames;
            if (frames.Count < 2)
                return;

            var (_, _, minEnd, tLast) = _viewport.SyncWithFrames(frames);

            EditorGUILayout.BeginHorizontal();
            _viewport.FollowLiveTail = EditorGUILayout.ToggleLeft("Follow Live", _viewport.FollowLiveTail, GUILayout.Width(95));
            GUI.enabled = tLast > minEnd + 1e-3;
            float sliderEnd = EditorGUILayout.Slider("Time End [s]", (float)_viewport.ViewEndTimeSec, (float)minEnd, (float)tLast);
            if (System.Math.Abs(sliderEnd - _viewport.ViewEndTimeSec) > 1e-3)
                _viewport.SetViewEndTime(sliderEnd, minEnd, tLast);
            GUI.enabled = true;

            if (GUILayout.Button("Reset View", GUILayout.Width(85)))
                _viewport.Reset(tLast);
            if (GUILayout.Button("+ Add Custom Graph", GUILayout.Width(145)))
                _customCharts.Add(new TelemetryCustomChart { Title = $"Custom Graph {_customCharts.Count + 1}" });
            EditorGUILayout.EndHorizontal();
        }

        private void DrawMetricsSummary(
            FlightDynamicsVehicle? vehicle,
            FlightPerformanceMetrics metrics,
            FlightLogRetentionSettings retention)
        {
            string sourceLabel = _repository.GetSourceLabel(vehicle);
            string px4Note = (vehicle != null && !vehicle.UseInternalFlightController)
                ? "\nNote: PX4 SITL Target* channels are populated when MAVLink ATTITUDE_TARGET (#83) / POSITION_TARGET_LOCAL_NED (#85) are received (e.g. UDP :8891 from unity_nav2_px4_bridge)."
                : string.Empty;
            EditorGUILayout.HelpBox(
                $"[{sourceLabel}] Frames: {metrics.SampleCount} (Cap {retention.MaxFramesInMemory:N0}) | Duration: {metrics.DurationSec:F2} s | " +
                $"Alt RMSE: {metrics.AltitudeRmseMeters:F3} m (Max Err {metrics.AltitudeMaxErrorMeters:F2} m) | " +
                $"Airspeed RMSE: {metrics.AirspeedRmseMps:F2} m/s | Roll RMSE: {metrics.RollRmseDeg:F2} deg | Pitch RMSE: {metrics.PitchRmseDeg:F2} deg | Max|Roll|: {metrics.MaxRollAbsDeg:F2} deg{px4Note}",
                MessageType.Info);
        }

        private void DrawCustomCharts(EditorWindow host, IReadOnlyList<TelemetryLogFrame> frames)
        {
            for (int c = 0; c < _customCharts.Count; c++)
            {
                var cfg = _customCharts[c];
                EditorGUILayout.BeginVertical(EditorStyles.helpBox);
                EditorGUILayout.BeginHorizontal();
                cfg.Title = EditorGUILayout.TextField(cfg.Title, GUILayout.Width(180));
                cfg.PendingAddIndex = EditorGUILayout.Popup(cfg.PendingAddIndex, TelemetryChannelRegistry.ChannelNames, GUILayout.Width(160));
                if (GUILayout.Button("+ Series", GUILayout.Width(70)) && !cfg.ChannelIndices.Contains(cfg.PendingAddIndex))
                    cfg.ChannelIndices.Add(cfg.PendingAddIndex);

                for (int s = cfg.ChannelIndices.Count - 1; s >= 0; s--)
                {
                    int chIdx = cfg.ChannelIndices[s];
                    if (GUILayout.Button($"{TelemetryChannelRegistry.Channels[chIdx].Name} ×", EditorStyles.miniButton))
                        cfg.ChannelIndices.RemoveAt(s);
                }

                GUILayout.FlexibleSpace();
                if (GUILayout.Button("Remove", GUILayout.Width(65)))
                {
                    _customCharts.RemoveAt(c);
                    EditorGUILayout.EndHorizontal();
                    EditorGUILayout.EndVertical();
                    break;
                }
                EditorGUILayout.EndHorizontal();

                if (cfg.ChannelIndices.Count > 0)
                {
                    var def = cfg.ToDefinition(TelemetryChannelRegistry.Channels);
                    TelemetryChartRenderer.Draw(host, def, frames, _viewport);
                }
                EditorGUILayout.EndVertical();
            }
        }
    }
}
