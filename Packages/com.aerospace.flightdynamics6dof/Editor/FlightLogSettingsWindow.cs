// Licensed under the Apache License, Version 2.0.
using System;
using System.IO;
using FlightDynamics6DoF.Unity;
using UnityEditor;
using UnityEngine;

namespace FlightDynamics6DoF.Editor
{
    /// <summary>
    /// Separate settings window for configuring memory buffer limits and disk retention policies
    /// for Assets/FlightLogs/ to prevent unbounded log growth.
    /// </summary>
    public class FlightLogSettingsWindow : EditorWindow
    {
        private FlightLogRetentionSettings _settings;
        private string _statusMessage = "";

        [MenuItem("Tools/FlightDynamics 6-DOF/Log & Retention Settings...", false, 20)]
        public static void ShowWindow()
        {
            var win = GetWindow<FlightLogSettingsWindow>("Flight Log Settings");
            win.minSize = new Vector2(460, 340);
            win.maxSize = new Vector2(640, 480);
            win.LoadCurrent();
            win.Show();
        }

        private void OnEnable()
        {
            LoadCurrent();
        }

        private void LoadCurrent()
        {
            _settings = FlightLogRetentionSettings.Load();
        }

        private void OnGUI()
        {
            EditorGUILayout.BeginVertical(EditorStyles.helpBox);
            GUILayout.Label("Flight Log Limits & Retention Policy", EditorStyles.boldLabel);
            EditorGUILayout.HelpBox(
                "Configure memory and disk (Assets/FlightLogs/) limits to prevent unbounded growth during long or repeated flights.",
                MessageType.Info);
            EditorGUILayout.EndVertical();

            EditorGUILayout.Space(6);

            EditorGUILayout.BeginVertical(EditorStyles.helpBox);
            GUILayout.Label("1. In-Memory Ring Buffer Limit", EditorStyles.boldLabel);
            _settings.MaxFramesInMemory = EditorGUILayout.IntSlider(
                new GUIContent("Max Recorded Frames", "At 50 Hz, 30,000 frames ≈ 10 minutes"),
                _settings.MaxFramesInMemory, 1000, 200000);
            double approxMinutesAt50Hz = _settings.MaxFramesInMemory / (50.0 * 60.0);
            EditorGUILayout.LabelField("Approx. Max Retention (at 50 Hz)", $"{approxMinutesAt50Hz:F1} min ({_settings.MaxFramesInMemory:N0} frames)");
            EditorGUILayout.EndVertical();

            EditorGUILayout.Space(6);

            EditorGUILayout.BeginVertical(EditorStyles.helpBox);
            GUILayout.Label("2. Disk Retention Limit (Assets/FlightLogs/)", EditorStyles.boldLabel);
            _settings.AutoPruneOldLogs = EditorGUILayout.ToggleLeft("Auto-Prune Old Log Files When Over Limit", _settings.AutoPruneOldLogs);
            GUI.enabled = _settings.AutoPruneOldLogs;
            _settings.MaxLogFilesOnDisk = EditorGUILayout.IntSlider("Max Retained Sessions (CSV)", _settings.MaxLogFilesOnDisk, 1, 500);
            _settings.MaxTotalLogSizeMb = EditorGUILayout.Slider("Max Total Log Size [MB]", _settings.MaxTotalLogSizeMb, 5f, 4096f);
            GUI.enabled = true;

            string logDir = FlightLogRetentionSettings.GetDefaultLogDirectory();
            var (csvCount, totalFiles, totalMb) = FlightLogRetentionSettings.MeasureDirectoryUsage(logDir);
            EditorGUILayout.Space(4);
            EditorGUILayout.LabelField("Current Assets/FlightLogs/ Usage", $"{csvCount} sessions ({totalFiles} files total) / {totalMb:F2} MB");
            EditorGUILayout.EndVertical();

            EditorGUILayout.Space(8);

            EditorGUILayout.BeginHorizontal();
            GUI.backgroundColor = new Color(0.55f, 0.95f, 0.65f);
            if (GUILayout.Button("Save & Apply Policy Now", GUILayout.Height(30)))
            {
                _settings.Save();
                int pruned = _settings.ApplyRetentionPolicy(logDir);
                ApplyMaxFramesToSceneVehicle(_settings.MaxFramesInMemory);
                AssetDatabase.Refresh();
                _statusMessage = pruned > 0
                    ? $"Settings saved; pruned {pruned} old session(s)."
                    : "Settings saved and retention policy applied.";
            }
            GUI.backgroundColor = Color.white;

            if (GUILayout.Button("Reset Defaults", GUILayout.Width(110), GUILayout.Height(30)))
            {
                _settings = new FlightLogRetentionSettings
                {
                    MaxFramesInMemory = FlightLogRetentionSettings.DefaultMaxFramesInMemory,
                    MaxLogFilesOnDisk = FlightLogRetentionSettings.DefaultMaxLogFilesOnDisk,
                    MaxTotalLogSizeMb = FlightLogRetentionSettings.DefaultMaxTotalLogSizeMb,
                    AutoPruneOldLogs = FlightLogRetentionSettings.DefaultAutoPruneOldLogs
                };
                _settings.Save();
                _statusMessage = "Restored default retention settings.";
            }
            EditorGUILayout.EndHorizontal();

            EditorGUILayout.BeginHorizontal();
            if (GUILayout.Button("Open Log Folder", GUILayout.Height(24)))
            {
                Directory.CreateDirectory(logDir);
                EditorUtility.RevealInFinder(logDir);
            }
            GUI.backgroundColor = new Color(1f, 0.6f, 0.6f);
            if (GUILayout.Button("Clear All Flight Logs", GUILayout.Height(24)))
            {
                if (EditorUtility.DisplayDialog("Delete All Flight Logs", "Delete all CSV and HTML reports in Assets/FlightLogs/?", "Delete", "Cancel"))
                {
                    if (Directory.Exists(logDir))
                    {
                        foreach (string file in Directory.GetFiles(logDir))
                        {
                            try { File.Delete(file); } catch { }
                        }
                        AssetDatabase.Refresh();
                    }
                    _statusMessage = "Deleted all logs in Assets/FlightLogs/.";
                }
            }
            GUI.backgroundColor = Color.white;
            EditorGUILayout.EndHorizontal();

            if (!string.IsNullOrEmpty(_statusMessage))
            {
                EditorGUILayout.Space(4);
                EditorGUILayout.HelpBox(_statusMessage, MessageType.None);
            }
        }

        private static void ApplyMaxFramesToSceneVehicle(int maxFrames)
        {
            var vehicle = FindFirstObjectByType<FlightDynamicsVehicle>();
            if (vehicle != null)
            {
                vehicle.TelemetryLogger.MaxFrames = maxFrames;
                vehicle.TelemetryLogger.EnforceMaxFrames();
            }
        }
    }
}
