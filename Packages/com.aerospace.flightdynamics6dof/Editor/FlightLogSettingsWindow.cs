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
            GUILayout.Label("비행 로그 상한선 및 보관 정책 설정 (Log & Retention Settings)", EditorStyles.boldLabel);
            EditorGUILayout.HelpBox(
                "장시간 비행 또는 반복 시뮬레이션 시 메모리와 디스크(Assets/FlightLogs/)가 과도하게 커지는 것을 방지하기 위한 상한선을 설정합니다.",
                MessageType.Info);
            EditorGUILayout.EndVertical();

            EditorGUILayout.Space(6);

            EditorGUILayout.BeginVertical(EditorStyles.helpBox);
            GUILayout.Label("1. 메모리 버퍼 상한선 (In-Memory Ring Buffer)", EditorStyles.boldLabel);
            _settings.MaxFramesInMemory = EditorGUILayout.IntSlider(
                new GUIContent("최대 기록 프레임 수", "50 Hz 기준 30,000 프레임 = 약 10분 분량"),
                _settings.MaxFramesInMemory, 1000, 200000);
            double approxMinutesAt50Hz = _settings.MaxFramesInMemory / (50.0 * 60.0);
            EditorGUILayout.LabelField("예상 최대 메모리 보존 시간 (50 Hz 기준)", $"약 {approxMinutesAt50Hz:F1} 분 ({_settings.MaxFramesInMemory:N0} frames)");
            EditorGUILayout.EndVertical();

            EditorGUILayout.Space(6);

            EditorGUILayout.BeginVertical(EditorStyles.helpBox);
            GUILayout.Label("2. 디스크 보관 상한선 (Assets/FlightLogs/)", EditorStyles.boldLabel);
            _settings.AutoPruneOldLogs = EditorGUILayout.ToggleLeft("상한 초과 시 오래된 로그 파일 자동 삭제 (Auto-Prune)", _settings.AutoPruneOldLogs);
            GUI.enabled = _settings.AutoPruneOldLogs;
            _settings.MaxLogFilesOnDisk = EditorGUILayout.IntSlider("최대 보관 비행 세션 수 (CSV 기준)", _settings.MaxLogFilesOnDisk, 1, 500);
            _settings.MaxTotalLogSizeMb = EditorGUILayout.Slider("최대 총 로그 용량 [MB]", _settings.MaxTotalLogSizeMb, 5f, 4096f);
            GUI.enabled = true;

            string logDir = FlightLogRetentionSettings.GetDefaultLogDirectory();
            var (csvCount, totalFiles, totalMb) = FlightLogRetentionSettings.MeasureDirectoryUsage(logDir);
            EditorGUILayout.Space(4);
            EditorGUILayout.LabelField("현재 Assets/FlightLogs/ 사용량", $"{csvCount}개 세션 (총 {totalFiles}개 파일) / {totalMb:F2} MB");
            EditorGUILayout.EndVertical();

            EditorGUILayout.Space(8);

            EditorGUILayout.BeginHorizontal();
            GUI.backgroundColor = new Color(0.55f, 0.95f, 0.65f);
            if (GUILayout.Button("설정 저장 및 상한 즉시 적용", GUILayout.Height(30)))
            {
                _settings.Save();
                int pruned = _settings.ApplyRetentionPolicy(logDir);
                ApplyMaxFramesToSceneVehicle(_settings.MaxFramesInMemory);
                AssetDatabase.Refresh();
                _statusMessage = pruned > 0
                    ? $"설정을 저장하고 오래된 {pruned}개 세션을 정리했습니다."
                    : "설정을 저장하고 상한선 정책을 적용했습니다.";
            }
            GUI.backgroundColor = Color.white;

            if (GUILayout.Button("기본값 복원", GUILayout.Width(100), GUILayout.Height(30)))
            {
                _settings = new FlightLogRetentionSettings
                {
                    MaxFramesInMemory = FlightLogRetentionSettings.DefaultMaxFramesInMemory,
                    MaxLogFilesOnDisk = FlightLogRetentionSettings.DefaultMaxLogFilesOnDisk,
                    MaxTotalLogSizeMb = FlightLogRetentionSettings.DefaultMaxTotalLogSizeMb,
                    AutoPruneOldLogs = FlightLogRetentionSettings.DefaultAutoPruneOldLogs
                };
                _settings.Save();
                _statusMessage = "기본 설정값으로 복원했습니다.";
            }
            EditorGUILayout.EndHorizontal();

            EditorGUILayout.BeginHorizontal();
            if (GUILayout.Button("로그 폴더 열기", GUILayout.Height(24)))
            {
                Directory.CreateDirectory(logDir);
                EditorUtility.RevealInFinder(logDir);
            }
            GUI.backgroundColor = new Color(1f, 0.6f, 0.6f);
            if (GUILayout.Button("모든 비행 로그 비우기", GUILayout.Height(24)))
            {
                if (EditorUtility.DisplayDialog("비행 로그 전체 삭제", "Assets/FlightLogs/ 폴더의 모든 CSV 및 HTML 리포트를 삭제하시겠습니까?", "삭제", "취소"))
                {
                    if (Directory.Exists(logDir))
                    {
                        foreach (string file in Directory.GetFiles(logDir))
                        {
                            try { File.Delete(file); } catch { }
                        }
                        AssetDatabase.Refresh();
                    }
                    _statusMessage = "Assets/FlightLogs/의 모든 로그를 삭제했습니다.";
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
