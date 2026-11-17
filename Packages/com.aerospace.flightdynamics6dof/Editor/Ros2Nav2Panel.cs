// Licensed under the Apache License, Version 2.0.
#nullable enable
using System;
using System.Diagnostics;
using FlightDynamics6DoF.Unity;
using UnityEditor;
using UnityEngine;

namespace FlightDynamics6DoF.Editor
{
    /// <summary>
    /// Step 6 Panel: ROS 2 Humble + Nav2 + RViz2 (noVNC) autonomous obstacle avoidance controls.
    /// </summary>
    internal sealed class Ros2Nav2Panel
    {
        private Process? _nav2Process;
        private float _offboardAltitude = 10.0f;
        private float _offboardRadius = 8.0f;
        private string _lastNote = "Idle";

        public void Draw(FlightDynamicsEngineeringWizard host)
        {
            GUILayout.Label("Step 6: ROS 2 Humble + Nav2 + RViz2 (Autonomous Obstacle Avoidance)", EditorStyles.boldLabel);

            bool rosUp = host.Infra.IsContainerRunning("unity-ros2-dds");
            EditorGUILayout.HelpBox(
                $"unity-ros2-dds: {(rosUp ? "UP (UDP :8888)" : "STOPPED")}  |  Nav2 Process: {(IsNav2Running() ? "RUNNING" : _lastNote)}",
                rosUp ? MessageType.Info : MessageType.Warning);

            EditorGUILayout.Space(4);
            DrawLidarAndObstacleSection(host);

            EditorGUILayout.Space(8);
            DrawNav2Section(host, rosUp);

            EditorGUILayout.Space(8);
            DrawOffboardOrbitSection(host);
        }

        private void DrawLidarAndObstacleSection(FlightDynamicsEngineeringWizard host)
        {
            EditorGUILayout.BeginVertical(EditorStyles.helpBox);
            GUILayout.Label("Unity 2D Raycast LiDAR (/scan) & Test Obstacle Course", EditorStyles.boldLabel);

            var vehicle = host.SceneVehicle;
            var lidar = vehicle != null ? vehicle.GetComponent<Ros2LidarAndObstacleCourse>() : null;

            if (vehicle == null)
            {
                EditorGUILayout.HelpBox("No FlightDynamicsVehicle in scene. It will attach automatically on Play/Spawn.", MessageType.None);
            }
            else if (lidar == null)
            {
                EditorGUILayout.BeginHorizontal();
                EditorGUILayout.LabelField("Ros2LidarAndObstacleCourse not attached (auto-attached on Play)");
                if (GUILayout.Button("Attach Now", GUILayout.Width(100)))
                    lidar = Undo.AddComponent<Ros2LidarAndObstacleCourse>(vehicle.gameObject);
                EditorGUILayout.EndHorizontal();
            }

            if (lidar != null)
            {
                EditorGUI.BeginChangeCheck();
                bool enableLidar = EditorGUILayout.Toggle("Enable 2D LiDAR (UDP :8890)", lidar.EnableLidar);
                bool genObstacles = EditorGUILayout.Toggle("Generate Test Obstacle Course", lidar.GenerateTestObstacles);
                int rayCount = EditorGUILayout.IntSlider("Ray Count (360°)", lidar.RayCount, 36, 360);
                float scanHz = EditorGUILayout.Slider("Scan Rate (Hz)", lidar.ScanRateHz, 2.0f, 20.0f);
                float maxRange = EditorGUILayout.Slider("Max Range (m)", lidar.MaxRangeMeters, 5.0f, 50.0f);
                if (EditorGUI.EndChangeCheck())
                {
                    Undo.RecordObject(lidar, "Update ROS 2 LiDAR Settings");
                    lidar.EnableLidar = enableLidar;
                    lidar.GenerateTestObstacles = genObstacles;
                    lidar.RayCount = rayCount;
                    lidar.ScanRateHz = scanHz;
                    lidar.MaxRangeMeters = maxRange;
                    EditorUtility.SetDirty(lidar);
                }

                EditorGUILayout.LabelField(
                    $"UDP Subscribers: {lidar.ActiveSubscribers}  |  Scans Sent: {lidar.TotalScansSent}",
                    EditorStyles.miniLabel);
            }
            EditorGUILayout.EndVertical();
        }

        private void DrawNav2Section(FlightDynamicsEngineeringWizard host, bool rosUp)
        {
            EditorGUILayout.BeginVertical(EditorStyles.helpBox);
            GUILayout.Label("Nav2 + RViz2 (noVNC Web Viewer)", EditorStyles.boldLabel);

            EditorGUILayout.BeginHorizontal();
            GUI.enabled = rosUp && !IsNav2Running();
            if (GUILayout.Button("▶ Start Nav2 + RViz2 (ros2-nav2)", GUILayout.Height(28)))
                StartNav2(host);

            GUI.enabled = IsNav2Running();
            if (GUILayout.Button("■ Stop Nav2", GUILayout.Height(28), GUILayout.Width(120)))
                StopNav2();

            GUI.enabled = true;
            if (GUILayout.Button("Open RViz2 (noVNC)", GUILayout.Height(28), GUILayout.Width(160)))
                Application.OpenURL("http://localhost:6080/vnc.html");
            EditorGUILayout.EndHorizontal();

            EditorGUILayout.BeginHorizontal();
            if (GUILayout.Button("List ROS 2 Topics (ros2-topics)"))
                host.Infra.RunScriptAction("ros2-topics", "Listing active ROS 2 topics...", () => EditorApplication.timeSinceStartup);
            EditorGUILayout.EndHorizontal();

            EditorGUILayout.HelpBox(
                "Click '2D Goal Pose' in RViz2 to set a target; the drone hovers at 2.5m and autonomously navigates around obstacles.\n" +
                "If localhost:6080 does not open, run `colima ssh -p flightdynamics -- -N -L 6080:127.0.0.1:6080` in a terminal.",
                MessageType.None);
            EditorGUILayout.EndVertical();
        }

        private void DrawOffboardOrbitSection(FlightDynamicsEngineeringWizard host)
        {
            EditorGUILayout.BeginVertical(EditorStyles.helpBox);
            GUILayout.Label("Built-in Offboard Orbit Trajectory Node (unity_px4_offboard_node.py)", EditorStyles.boldLabel);
            EditorGUILayout.BeginHorizontal();
            _offboardAltitude = EditorGUILayout.FloatField("Alt (m)", _offboardAltitude);
            _offboardRadius = EditorGUILayout.FloatField("Radius (m)", _offboardRadius);
            if (GUILayout.Button("Start Offboard"))
            {
                var launcher = UnityEngine.Object.FindFirstObjectByType<Px4SitlProcessLauncher>();
                if (launcher != null) launcher.StartRos2OffboardNode(_offboardAltitude, _offboardRadius);
                else host.Infra.RunScriptAction($"ros2-offboard --altitude {_offboardAltitude:F1} --radius {_offboardRadius:F1}", "Starting ROS 2 Offboard node...", () => EditorApplication.timeSinceStartup);
            }
            if (GUILayout.Button("Stop Offboard"))
                UnityEngine.Object.FindFirstObjectByType<Px4SitlProcessLauncher>()?.StopRos2OffboardNode();
            EditorGUILayout.EndHorizontal();
            EditorGUILayout.EndVertical();
        }

        private bool IsNav2Running()
        {
            try { return _nav2Process != null && !_nav2Process.HasExited; }
            catch { return false; }
        }

        private void StartNav2(FlightDynamicsEngineeringWizard host)
        {
            StopNav2();
            string scriptPath = SitlInfraController.GetManagerScriptPath();
            string wsRoot = SitlInfraController.GetWorkspaceRoot();
            try
            {
                var psi = new ProcessStartInfo
                {
                    FileName = "/bin/bash",
                    Arguments = $"-lc \"{host.Infra.GetBackendEnvPrefix()}'{scriptPath}' ros2-nav2\"",
                    WorkingDirectory = wsRoot,
                    UseShellExecute = false,
                    RedirectStandardOutput = true,
                    RedirectStandardError = true,
                    CreateNoWindow = true
                };
                _nav2Process = Process.Start(psi);
                if (_nav2Process != null)
                {
                    _nav2Process.BeginOutputReadLine();
                    _nav2Process.BeginErrorReadLine();
                    _lastNote = "Running (noVNC :6080)";
                }
            }
            catch (Exception ex)
            {
                _lastNote = $"Failed: {ex.Message}";
            }
        }

        internal void StopNav2()
        {
            if (_nav2Process == null) return;
            try
            {
                if (!_nav2Process.HasExited) _nav2Process.Kill();
                _nav2Process.Dispose();
            }
            catch { }
            _nav2Process = null;
            _lastNote = "Stopped";
        }
    }
}

