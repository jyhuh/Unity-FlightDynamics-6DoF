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
            GUILayout.Label("Step 2.1: SITL Runtime Infrastructure One-Click Controls", EditorStyles.boldLabel);
            EditorGUILayout.HelpBox(
                "Start the Colima VM, PX4 SITL (unity-px4-sitl, isolated rootfs), and ROS 2 DDS (unity-ros2-dds), or shut down the entire stack including the VM.",
                MessageType.Info);

            EditorGUILayout.BeginHorizontal();
            GUI.backgroundColor = new Color(0.45f, 0.95f, 0.55f);
            if (GUILayout.Button($"▶ Start All Infra ({airframeModel})", GUILayout.Height(36)))
            {
                exportAirframeParams();
                infra.RunScriptAction($"up {airframeModel}", $"Starting full SITL infrastructure for '{airframeModel}'...", () => EditorApplication.timeSinceStartup);
            }

            GUI.backgroundColor = new Color(1f, 0.5f, 0.5f);
            if (GUILayout.Button("■ Stop All (Shutdown VM)", GUILayout.Height(36)))
            {
                infra.RunScriptAction("vm-stop", "Stopping containers and shutting down Colima VM...", () => EditorApplication.timeSinceStartup);
            }

            GUI.backgroundColor = Color.white;
            if (GUILayout.Button("↻ Refresh Status", GUILayout.Width(125), GUILayout.Height(36)))
            {
                infra.TriggerStatusQuery(EditorApplication.timeSinceStartup);
            }
            EditorGUILayout.EndHorizontal();

            EditorGUILayout.Space(4);
            EditorGUILayout.LabelField("Last Action:", infra.LastStatusSummary, EditorStyles.miniLabel);
            EditorGUILayout.EndVertical();

            EditorGUILayout.Space(6);

            _foldAdvancedInfra = EditorGUILayout.Foldout(_foldAdvancedInfra, "Advanced: Individual VM / Container Controls & Live Console Log", true, EditorStyles.foldoutHeader);
            if (_foldAdvancedInfra)
            {
                EditorGUILayout.BeginVertical(EditorStyles.helpBox);
                infra.Backend = (ContainerRuntimeBackend)EditorGUILayout.EnumPopup("Runtime Backend", infra.Backend);

                EditorGUILayout.BeginHorizontal();
                if (GUILayout.Button("Start/Restart PX4"))
                {
                    exportAirframeParams();
                    infra.RunScriptAction($"start-px4 {airframeModel}", $"Restarting unity-px4-sitl ({airframeModel})...", () => EditorApplication.timeSinceStartup);
                }
                if (GUILayout.Button("Stop PX4")) infra.RunScriptAction("stop-px4", "Stopping unity-px4-sitl...", () => EditorApplication.timeSinceStartup);
                if (GUILayout.Button("Start/Restart ROS 2")) infra.RunScriptAction("start-ros2", "Restarting unity-ros2-dds...", () => EditorApplication.timeSinceStartup);
                if (GUILayout.Button("Stop ROS 2")) infra.RunScriptAction("stop-ros2", "Stopping unity-ros2-dds...", () => EditorApplication.timeSinceStartup);
                if (GUILayout.Button("Reset rootfs Cache")) infra.RunScriptAction("reset-rootfs", "Clearing isolated rootfs_* caches...", () => EditorApplication.timeSinceStartup);
                EditorGUILayout.EndHorizontal();

                EditorGUILayout.BeginHorizontal();
                if (GUILayout.Button("Check VM/containerd")) infra.RunScriptAction("vm-status", "Checking VM & containerd health...", () => EditorApplication.timeSinceStartup);
                if (GUILayout.Button("Repair containerd")) infra.RunScriptAction("vm-repair", "Repairing containerd service...", () => EditorApplication.timeSinceStartup);
                if (GUILayout.Button("Start Colima VM")) infra.RunScriptAction("vm-start", "Starting Colima VM...", () => EditorApplication.timeSinceStartup);
                if (GUILayout.Button("Stop Containers Only (down)")) infra.RunScriptAction("down", "Stopping containers (keeping VM warm)...", () => EditorApplication.timeSinceStartup);
                if (GUILayout.Button("View Logs in Terminal")) infra.OpenLogsInTerminal();
                EditorGUILayout.EndHorizontal();

                EditorGUILayout.Space(4);
                EditorGUILayout.BeginHorizontal();
                GUILayout.Label("Infra Console Log", EditorStyles.miniBoldLabel);
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

