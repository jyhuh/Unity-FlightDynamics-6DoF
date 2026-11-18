// Licensed under the Apache License, Version 2.0.
#nullable enable
using System;
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
    /// Repository managing live, disk-loaded, and offline-demo telemetry buffers plus Assets/FlightLogs persistence.
    /// </summary>
    internal sealed class TelemetryLogRepository
    {
        private readonly FlightTelemetryLogger _offlinePreviewLogger = new FlightTelemetryLogger();
        private readonly FlightTelemetryLogger _loadedDiskLogger = new FlightTelemetryLogger();

        public string[] SavedLogPaths { get; private set; } = Array.Empty<string>();
        public string[] SavedLogLabels { get; private set; } = Array.Empty<string>();
        public int SelectedSavedLogIndex { get; set; } = -1;
        public string LoadedDiskLogPath { get; private set; } = "";
        public bool DiskLogListScanned { get; private set; }

        public FlightTelemetryLogger GetActiveLogger(FlightDynamicsVehicle? sceneVehicle)
        {
            if (sceneVehicle != null && sceneVehicle.TelemetryLogger.Frames.Count > 0)
                return sceneVehicle.TelemetryLogger;
            if (_offlinePreviewLogger.Frames.Count > 0)
                return _offlinePreviewLogger;
            if (!DiskLogListScanned)
                RefreshSavedFlightLogList(autoLoadLatestIfEmpty: true);
            if (_loadedDiskLogger.Frames.Count > 0)
                return _loadedDiskLogger;
            return sceneVehicle != null ? sceneVehicle.TelemetryLogger : _offlinePreviewLogger;
        }

        public string GetSourceLabel(FlightDynamicsVehicle? sceneVehicle)
        {
            if (sceneVehicle != null && sceneVehicle.TelemetryLogger.Frames.Count > 0)
                return "Live Simulation Buffer";
            if (!string.IsNullOrEmpty(LoadedDiskLogPath))
                return $"Disk Log: {Path.GetFileName(LoadedDiskLogPath)}";
            return "Offline Demo / Empty Buffer";
        }

        public void ClearBuffers(FlightTelemetryLogger activeLogger)
        {
            activeLogger.Clear();
            _loadedDiskLogger.Clear();
            LoadedDiskLogPath = "";
        }

        public void RefreshSavedFlightLogList(bool autoLoadLatestIfEmpty)
        {
            DiskLogListScanned = true;
            string logDir = FlightLogRetentionSettings.GetDefaultLogDirectory();
            if (!Directory.Exists(logDir))
            {
                SavedLogPaths = Array.Empty<string>();
                SavedLogLabels = Array.Empty<string>();
                SelectedSavedLogIndex = -1;
                return;
            }

            var files = new DirectoryInfo(logDir).GetFiles("*.csv", SearchOption.TopDirectoryOnly)
                .OrderByDescending(f => f.LastWriteTimeUtc)
                .ToArray();

            SavedLogPaths = new string[files.Length];
            SavedLogLabels = new string[files.Length];
            for (int i = 0; i < files.Length; i++)
            {
                SavedLogPaths[i] = files[i].FullName;
                double kb = files[i].Length / 1024.0;
                SavedLogLabels[i] = $"{files[i].Name} ({kb:F1} KB)";
            }

            if (files.Length > 0)
            {
                int prevIdx = Array.IndexOf(SavedLogPaths, LoadedDiskLogPath);
                SelectedSavedLogIndex = prevIdx >= 0 ? prevIdx : 0;
                if (autoLoadLatestIfEmpty && _loadedDiskLogger.Frames.Count == 0)
                    LoadDiskLogAtIndex(SelectedSavedLogIndex);
            }
            else
            {
                SelectedSavedLogIndex = -1;
            }
        }

        public void LoadDiskLogAtIndex(int index)
        {
            if (index < 0 || index >= SavedLogPaths.Length) return;
            string path = SavedLogPaths[index];
            var retention = FlightLogRetentionSettings.Load();
            _loadedDiskLogger.MaxFrames = retention.MaxFramesInMemory;
            int count = _loadedDiskLogger.LoadFromCsvFile(path);
            if (count > 0)
                LoadedDiskLogPath = path;
        }

        public void SaveManualSnapshot(FlightTelemetryLogger logger, string aircraftName, FlightLogRetentionSettings retention)
        {
            string logDir = FlightLogRetentionSettings.GetDefaultLogDirectory();
            Directory.CreateDirectory(logDir);
            string stamp = DateTime.Now.ToString("yyyyMMdd_HHmmss");
            string csvPath = Path.Combine(logDir, $"manual_{stamp}.csv");
            string htmlPath = Path.Combine(logDir, $"manual_report_{stamp}.html");
            logger.ExportToCsvFile(csvPath);
            logger.ExportToAcademicHtmlReportFile(htmlPath, $"FlightDynamics 6-DOF Report - {aircraftName}");
            retention.ApplyRetentionPolicy(logDir);
            AssetDatabase.Refresh();
            RefreshSavedFlightLogList(autoLoadLatestIfEmpty: false);
            Debug.Log($"[FlightDynamics6DoF] Saved logs to Assets/FlightLogs/: {Path.GetFileName(csvPath)}, {Path.GetFileName(htmlPath)}");
        }

        public void GenerateClosedLoopFixedWingDemoLog()
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
