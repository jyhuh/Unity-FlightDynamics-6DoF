// Licensed under the Apache License, Version 2.0.
#nullable enable
using System;
using FlightDynamics6DoF.Unity;
using UnityEditor;
using UnityEngine;

namespace FlightDynamics6DoF.Editor
{
    /// <summary>
    /// Step 2: SITL runtime infrastructure controls and console log viewer.
    /// </summary>
    internal sealed class InfraControlPanel
    {
        private Vector2 _consoleScrollPos;
        private bool _foldAdvancedInfra = false;
        private bool _autoScrollConsole = true;

        public void Draw(SitlInfraController infra, string airframeModel, Action exportAirframeParams)
        {
            EditorGUILayout.BeginVertical(EditorStyles.helpBox);
            GUILayout.Label("Step 2.1: SITL 런타임 인프라 원클릭 제어", EditorStyles.boldLabel);
            EditorGUILayout.HelpBox(
                "원클릭 버튼으로 Colima VM, PX4 SITL(unity-px4-sitl, 기체별 격리 rootfs) 및 ROS 2 DDS(unity-ros2-dds)를 한 번에 시작하거나 중지할 수 있습니다.",
                MessageType.Info);

            EditorGUILayout.BeginHorizontal();
            GUI.backgroundColor = new Color(0.45f, 0.95f, 0.55f);
            if (GUILayout.Button($"▶ 인프라 전체 시작 ({airframeModel})", GUILayout.Height(36)))
            {
                exportAirframeParams();
                infra.RunScriptAction($"up {airframeModel}", $"Starting full SITL infrastructure for '{airframeModel}'...", () => EditorApplication.timeSinceStartup);
            }

            GUI.backgroundColor = new Color(1f, 0.5f, 0.5f);
            if (GUILayout.Button("■ 인프라 전체 중지 (컨테이너 정지)", GUILayout.Height(36)))
            {
                infra.RunScriptAction("down", "Stopping unity-px4-sitl and unity-ros2-dds containers...", () => EditorApplication.timeSinceStartup);
            }

            GUI.backgroundColor = Color.white;
            if (GUILayout.Button("↻ 상태 새로고침", GUILayout.Width(120), GUILayout.Height(36)))
            {
                infra.TriggerStatusQuery(EditorApplication.timeSinceStartup);
            }
            EditorGUILayout.EndHorizontal();

            EditorGUILayout.Space(4);
            EditorGUILayout.LabelField("최근 작업:", infra.LastStatusSummary, EditorStyles.miniLabel);
            EditorGUILayout.EndVertical();

            EditorGUILayout.Space(6);

            _foldAdvancedInfra = EditorGUILayout.Foldout(_foldAdvancedInfra, "고급: 개별 VM / 컨테이너 제어 및 실시간 콘솔 로그", true, EditorStyles.foldoutHeader);
            if (_foldAdvancedInfra)
            {
                EditorGUILayout.BeginVertical(EditorStyles.helpBox);
                infra.Backend = (ContainerRuntimeBackend)EditorGUILayout.EnumPopup("Runtime Backend", infra.Backend);

                EditorGUILayout.BeginHorizontal();
                if (GUILayout.Button("PX4만 시작/재시작"))
                {
                    exportAirframeParams();
                    infra.RunScriptAction($"start-px4 {airframeModel}", $"Restarting unity-px4-sitl ({airframeModel})...", () => EditorApplication.timeSinceStartup);
                }
                if (GUILayout.Button("PX4 중지")) infra.RunScriptAction("stop-px4", "Stopping unity-px4-sitl...", () => EditorApplication.timeSinceStartup);
                if (GUILayout.Button("ROS 2 시작/재시작")) infra.RunScriptAction("start-ros2", "Restarting unity-ros2-dds...", () => EditorApplication.timeSinceStartup);
                if (GUILayout.Button("ROS 2 중지")) infra.RunScriptAction("stop-ros2", "Stopping unity-ros2-dds...", () => EditorApplication.timeSinceStartup);
                if (GUILayout.Button("rootfs 캐시 초기화")) infra.RunScriptAction("reset-rootfs", "Clearing isolated rootfs_* caches...", () => EditorApplication.timeSinceStartup);
                EditorGUILayout.EndHorizontal();

                EditorGUILayout.BeginHorizontal();
                if (GUILayout.Button("VM/containerd 상태 점검")) infra.RunScriptAction("vm-status", "Checking VM & containerd health...", () => EditorApplication.timeSinceStartup);
                if (GUILayout.Button("containerd 복구")) infra.RunScriptAction("vm-repair", "Repairing containerd service...", () => EditorApplication.timeSinceStartup);
                if (GUILayout.Button("Colima VM 시작")) infra.RunScriptAction("vm-start", "Starting Colima VM...", () => EditorApplication.timeSinceStartup);
                if (GUILayout.Button("Colima VM 완전 종료")) infra.RunScriptAction("vm-stop", "Shutting down Colima VM...", () => EditorApplication.timeSinceStartup);
                if (GUILayout.Button("터미널에서 로그 보기")) infra.OpenLogsInTerminal();
                EditorGUILayout.EndHorizontal();

                EditorGUILayout.Space(4);
                EditorGUILayout.BeginHorizontal();
                GUILayout.Label("인프라 콘솔 로그", EditorStyles.miniBoldLabel);
                _autoScrollConsole = EditorGUILayout.ToggleLeft("Auto Scroll", _autoScrollConsole, GUILayout.Width(90));
                GUILayout.FlexibleSpace();
                if (GUILayout.Button("Clear", GUILayout.Width(60))) infra.ClearConsole();
                if (GUILayout.Button("Copy", GUILayout.Width(60))) EditorGUIUtility.systemCopyBuffer = infra.ConsoleText;
                EditorGUILayout.EndHorizontal();

                var textStyle = new GUIStyle(EditorStyles.textArea)
                {
                    fontSize = 11,
                    wordWrap = true,
                    normal = { textColor = new Color(0.85f, 0.95f, 0.85f), background = Texture2D.blackTexture }
                };
                _consoleScrollPos = EditorGUILayout.BeginScrollView(_consoleScrollPos, GUILayout.Height(180));
                EditorGUILayout.TextArea(infra.ConsoleText, textStyle, GUILayout.ExpandHeight(true));
                if (_autoScrollConsole && Event.current.type == EventType.Repaint)
                    _consoleScrollPos.y = float.MaxValue;
                EditorGUILayout.EndScrollView();
                EditorGUILayout.EndVertical();
            }
        }
    }
}

