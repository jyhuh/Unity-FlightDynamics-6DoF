// Licensed under the Apache License, Version 2.0.
#nullable enable
using System;
using System.Collections.Concurrent;
using System.Collections.Generic;
using System.Diagnostics;
using System.IO;
using System.Text;
using System.Threading.Tasks;
using UnityEditor;
using UnityEngine;
using FlightDynamics6DoF.Core.Specs;
using FlightDynamics6DoF.Unity;
using Debug = UnityEngine.Debug;

namespace FlightDynamics6DoF.Editor
{
    /// <summary>
    /// Holds parsed inspection status for an individual container (e.g., `unity-px4-sitl`, `unity-ros2-dds`).
    /// </summary>
    public class ContainerStateInfo
    {
        public string Name = "";
        public string RawStatus = "STOPPED";
        public string Image = "";
        public string Ports = "";
        public bool IsRunning => !string.IsNullOrEmpty(RawStatus) && RawStatus.StartsWith("Up", StringComparison.OrdinalIgnoreCase);
    }

    /// <summary>
    /// Dedicated Unity EditorWindow for complete lifecycle management and live state monitoring
    /// of the non-Docker container stack (PX4 SITL `unity-px4-sitl` and ROS 2 Humble `unity-ros2-dds`),
    /// Colima macOS virtual machine, and internal systemd `containerd.service` runtime daemon.
    /// Accessible via menu: Tools > FlightDynamics 6-DOF > Container &amp; VM Manager
    /// </summary>
    public class Px4ContainerManagerWindow : EditorWindow
    {
        private ContainerRuntimeBackend _backend = ContainerRuntimeBackend.AutoDetect;
        private string _airframeModel = "iris";
        private string _customPxhCommand = "commander takeoff";
        private float _offboardAltitude = 10.0f;
        private float _offboardRadius = 8.0f;

        private Vector2 _windowScrollPos;
        private Vector2 _consoleScrollPos;
        private bool _autoScrollConsole = true;

        private bool _foldStatusDashboard = true;
        private bool _foldLifecycle = true;
        private bool _foldVmRuntime = true;
        private bool _foldPxh = true;
        private bool _foldRos2 = false;
        private bool _foldConsole = true;

        // Container & VM Live Status Cache
        private string _vmStatus = "UNKNOWN"; // RUNNING, STOPPED, UNKNOWN
        private string _containerdStatus = "UNKNOWN"; // ACTIVE, DEGRADED, INACTIVE, UNKNOWN
        private readonly Dictionary<string, ContainerStateInfo> _containers = new Dictionary<string, ContainerStateInfo>(StringComparer.OrdinalIgnoreCase);
        private double _lastStatusQueryTime;
        private bool _isQueryingStatus = false;
        private bool _autoRefreshStatus = true;
        private const double RefreshIntervalSeconds = 3.5;

        private static readonly StringBuilder _consoleLogBuffer = new StringBuilder();
        private static readonly ConcurrentQueue<string> _pendingLogLines = new ConcurrentQueue<string>();
        private static readonly ConcurrentQueue<Action> _mainThreadActions = new ConcurrentQueue<Action>();
        private static string _lastStatusSummary = "Ready";
        private static bool _isOperationRunning = false;

        [MenuItem("Tools/FlightDynamics 6-DOF/Container & VM Manager", false, 15)]
        [MenuItem("Window/Aerospace/Container & VM Manager", false, 20)]
        public static void ShowWindow()
        {
            var win = GetWindow<Px4ContainerManagerWindow>("Container & VM Manager");
            win.minSize = new Vector2(620, 720);
            win.Show();
        }

        private void OnEnable()
        {
            EditorApplication.update += HandleEditorUpdate;
            TriggerAsyncStatusQuery();
        }

        private void OnDisable()
        {
            EditorApplication.update -= HandleEditorUpdate;
        }

        private void HandleEditorUpdate()
        {
            // Run queued main-thread actions
            while (_mainThreadActions.TryDequeue(out var action))
            {
                action?.Invoke();
            }

            bool hasNewLogs = false;
            while (_pendingLogLines.TryDequeue(out string? line))
            {
                if (!string.IsNullOrEmpty(line))
                {
                    _consoleLogBuffer.AppendLine(line);
                    _lastStatusSummary = line;
                    hasNewLogs = true;
                }
            }

            // Periodic auto status polling
            if (_autoRefreshStatus && !_isOperationRunning && !_isQueryingStatus)
            {
                if (EditorApplication.timeSinceStartup - _lastStatusQueryTime >= RefreshIntervalSeconds)
                {
                    TriggerAsyncStatusQuery();
                }
            }

            if (hasNewLogs)
            {
                Repaint();
            }
        }

        private string GetWorkspaceRoot()
        {
            var launcher = FindFirstObjectByType<Px4SitlProcessLauncher>();
            if (launcher != null && !string.IsNullOrEmpty(launcher.EffectiveWorkspaceRoot))
                return launcher.EffectiveWorkspaceRoot;

            try
            {
                string candidate = Path.GetFullPath(Path.Combine(Application.dataPath, ".."));
                if (Directory.Exists(candidate)) return candidate;
            }
            catch { }

            return Directory.GetCurrentDirectory();
        }

        private string GetManagerScriptPath()
        {
            return Path.Combine(GetWorkspaceRoot(), "scripts", "px4-sitl.sh");
        }

        private string GetBackendEnvPrefix()
        {
            return _backend switch
            {
                ContainerRuntimeBackend.ColimaContainerd => "CONTAINER_BACKEND=colima ",
                ContainerRuntimeBackend.AppleContainer => "CONTAINER_BACKEND=apple ",
                _ => "CONTAINER_BACKEND=auto "
            };
        }

        private void TriggerAsyncStatusQuery()
        {
            if (_isQueryingStatus) return;
            _isQueryingStatus = true;
            _lastStatusQueryTime = EditorApplication.timeSinceStartup;

            string scriptPath = GetManagerScriptPath();
            string wsRoot = GetWorkspaceRoot();
            string envPrefix = GetBackendEnvPrefix();

            Task.Run(() =>
            {
                string newVmStatus = "STOPPED";
                string newContainerdStatus = "INACTIVE";
                var newContainers = new Dictionary<string, ContainerStateInfo>(StringComparer.OrdinalIgnoreCase);

                try
                {
                    if (File.Exists(scriptPath))
                    {
                        var psi = new ProcessStartInfo
                        {
                            FileName = "/bin/bash",
                            Arguments = $"-lc \"{envPrefix}'{scriptPath}' status-info\"",
                            WorkingDirectory = wsRoot,
                            UseShellExecute = false,
                            RedirectStandardOutput = true,
                            RedirectStandardError = true,
                            CreateNoWindow = true
                        };

                        using var proc = Process.Start(psi);
                        if (proc != null)
                        {
                            string output = proc.StandardOutput.ReadToEnd();
                            proc.WaitForExit(3000);

                            using var reader = new StringReader(output);
                            string? line;
                            while ((line = reader.ReadLine()) != null)
                            {
                                line = line.Trim();
                                if (line.StartsWith("VM_STATUS="))
                                {
                                    newVmStatus = line.Substring("VM_STATUS=".Length).Trim();
                                }
                                else if (line.StartsWith("CONTAINERD_STATUS="))
                                {
                                    newContainerdStatus = line.Substring("CONTAINERD_STATUS=".Length).Trim();
                                }
                                else if (line.Contains("|"))
                                {
                                    string[] parts = line.Split('|');
                                    if (parts.Length >= 2)
                                    {
                                        string name = parts[0].Trim();
                                        if (!string.IsNullOrEmpty(name))
                                        {
                                            var info = new ContainerStateInfo
                                            {
                                                Name = name,
                                                RawStatus = parts[1].Trim(),
                                                Image = parts.Length > 2 ? parts[2].Trim() : "",
                                                Ports = parts.Length > 3 ? parts[3].Trim() : ""
                                            };
                                            newContainers[name] = info;
                                        }
                                    }
                                }
                            }
                        }
                    }
                }
                catch
                {
                    newVmStatus = "ERROR";
                    newContainerdStatus = "ERROR";
                }
                finally
                {
                    _mainThreadActions.Enqueue(() =>
                    {
                        _vmStatus = newVmStatus;
                        _containerdStatus = newContainerdStatus;
                        _containers.Clear();
                        foreach (var kvp in newContainers)
                            _containers[kvp.Key] = kvp.Value;
                        _isQueryingStatus = false;
                        Repaint();
                    });
                }
            });
        }

        private void OnGUI()
        {
            _windowScrollPos = EditorGUILayout.BeginScrollView(_windowScrollPos);

            DrawHeaderAndStatusDashboard();
            EditorGUILayout.Space(6);

            DrawContainerInspectionTable();
            EditorGUILayout.Space(6);

            DrawContainerLifecycleSection();
            EditorGUILayout.Space(6);

            DrawColimaVmAndRuntimeSection();
            EditorGUILayout.Space(6);

            DrawPxhCommandSection();
            EditorGUILayout.Space(6);

            DrawRos2Section();
            EditorGUILayout.Space(6);

            DrawLiveConsoleSection();

            EditorGUILayout.EndScrollView();
        }

        private void DrawHeaderAndStatusDashboard()
        {
            EditorGUILayout.BeginVertical(EditorStyles.helpBox);
            EditorGUILayout.BeginHorizontal();
            GUILayout.Label("FlightDynamics 6-DOF: Container & VM Manager", EditorStyles.boldLabel);
            GUILayout.FlexibleSpace();
            _autoRefreshStatus = EditorGUILayout.ToggleLeft("Auto Poll", _autoRefreshStatus, GUILayout.Width(80));
            GUI.enabled = !_isQueryingStatus;
            if (GUILayout.Button(_isQueryingStatus ? "Checking..." : "↻ Refresh Status", GUILayout.Width(110), GUILayout.Height(20)))
            {
                TriggerAsyncStatusQuery();
            }
            GUI.enabled = true;
            EditorGUILayout.EndHorizontal();

            EditorGUILayout.LabelField("Zero-Docker macOS Runtime: Colima (containerd + nerdctl) / Apple native 'container'", EditorStyles.miniLabel);

            EditorGUILayout.Space(4);

            // 4-Card Quick Status Overview
            EditorGUILayout.BeginHorizontal();

            // 1. Colima VM Status Card
            bool vmRunning = _vmStatus.Equals("RUNNING", StringComparison.OrdinalIgnoreCase);
            DrawStatusCard("Colima macOS VM", vmRunning ? "RUNNING" : _vmStatus, vmRunning ? new Color(0.2f, 0.8f, 0.3f) : new Color(0.9f, 0.3f, 0.3f));

            // 2. containerd Daemon Card
            bool cdActive = _containerdStatus.Equals("ACTIVE", StringComparison.OrdinalIgnoreCase);
            Color cdCol = cdActive ? new Color(0.2f, 0.8f, 0.3f) : (_containerdStatus.Equals("DEGRADED", StringComparison.OrdinalIgnoreCase) ? new Color(1f, 0.7f, 0.2f) : new Color(0.9f, 0.3f, 0.3f));
            DrawStatusCard("containerd Daemon", _containerdStatus, cdCol);

            // 3. unity-px4-sitl Container Card
            bool px4Running = _containers.TryGetValue("unity-px4-sitl", out var px4Info) && px4Info.IsRunning;
            DrawStatusCard("unity-px4-sitl", px4Running ? "UP (TCP :4560)" : "STOPPED", px4Running ? new Color(0.2f, 0.8f, 0.3f) : new Color(0.8f, 0.4f, 0.4f));

            // 4. unity-ros2-dds Container Card
            bool ros2Running = _containers.TryGetValue("unity-ros2-dds", out var ros2Info) && ros2Info.IsRunning;
            DrawStatusCard("unity-ros2-dds", ros2Running ? "UP (UDP :8888)" : "STOPPED", ros2Running ? new Color(0.2f, 0.8f, 0.3f) : new Color(0.8f, 0.4f, 0.4f));

            EditorGUILayout.EndHorizontal();

            EditorGUILayout.Space(4);
            EditorGUILayout.BeginHorizontal();
            _backend = (ContainerRuntimeBackend)EditorGUILayout.EnumPopup("Runtime Backend:", _backend);
            EditorGUILayout.LabelField("Model:", _airframeModel, GUILayout.Width(100));
            EditorGUILayout.EndHorizontal();

            EditorGUILayout.BeginHorizontal();
            EditorGUILayout.LabelField("Recent Activity:", _lastStatusSummary, EditorStyles.wordWrappedMiniLabel);
            EditorGUILayout.EndHorizontal();

            EditorGUILayout.EndVertical();
        }

        private static void DrawStatusCard(string title, string statusText, Color statusColor)
        {
            EditorGUILayout.BeginVertical(EditorStyles.helpBox, GUILayout.MinWidth(135));
            GUILayout.Label(title, EditorStyles.miniBoldLabel);
            var prevCol = GUI.contentColor;
            GUI.contentColor = statusColor;
            GUILayout.Label($"● {statusText}", EditorStyles.boldLabel);
            GUI.contentColor = prevCol;
            EditorGUILayout.EndVertical();
        }

        private void DrawContainerInspectionTable()
        {
            _foldStatusDashboard = EditorGUILayout.Foldout(_foldStatusDashboard, "Live Container Status & Network Bindings", true, EditorStyles.foldoutHeader);
            if (!_foldStatusDashboard) return;

            EditorGUILayout.BeginVertical(EditorStyles.helpBox);

            // Table Header
            EditorGUILayout.BeginHorizontal(EditorStyles.toolbar);
            GUILayout.Label("Container Name", EditorStyles.boldLabel, GUILayout.Width(140));
            GUILayout.Label("State / Status", EditorStyles.boldLabel, GUILayout.Width(110));
            GUILayout.Label("Network Ports & Bindings", EditorStyles.boldLabel, GUILayout.ExpandWidth(true));
            GUILayout.Label("Actions", EditorStyles.boldLabel, GUILayout.Width(120));
            EditorGUILayout.EndHorizontal();

            // Row 1: unity-px4-sitl
            _containers.TryGetValue("unity-px4-sitl", out var px4Info);
            DrawContainerRow(
                "unity-px4-sitl",
                px4Info != null && px4Info.IsRunning,
                px4Info != null ? px4Info.RawStatus : "STOPPED / NOT FOUND",
                "TCP 4560 (Unity Lockstep HIL) | UDP 14550 (QGC) | UDP 14540 (Offboard)",
                () => RunScriptAction($"up {_airframeModel}", $"Restarting unity-px4-sitl ({_airframeModel})..."),
                () => OpenLogsInTerminal()
            );

            // Row 2: unity-ros2-dds
            _containers.TryGetValue("unity-ros2-dds", out var ros2Info);
            DrawContainerRow(
                "unity-ros2-dds",
                ros2Info != null && ros2Info.IsRunning,
                ros2Info != null ? ros2Info.RawStatus : "STOPPED / NOT FOUND",
                "UDP 8888 (MicroXRCEAgent uORB <-> ROS 2 Humble Topics)",
                () => RunScriptAction($"up {_airframeModel}", $"Restarting unity-ros2-dds stack..."),
                () => RunScriptAction("ros2-topics", "Listing active ROS 2 topics...")
            );

            // Any extra containers discovered via nerdctl ps
            foreach (var kvp in _containers)
            {
                if (kvp.Key.Equals("unity-px4-sitl", StringComparison.OrdinalIgnoreCase) ||
                    kvp.Key.Equals("unity-ros2-dds", StringComparison.OrdinalIgnoreCase))
                    continue;

                var c = kvp.Value;
                DrawContainerRow(
                    c.Name,
                    c.IsRunning,
                    c.RawStatus,
                    string.IsNullOrEmpty(c.Ports) ? c.Image : c.Ports,
                    null,
                    null
                );
            }

            EditorGUILayout.EndVertical();
        }

        private static void DrawContainerRow(string name, bool isRunning, string statusText, string portsText, Action? onPrimaryAction, Action? onSecondaryAction)
        {
            EditorGUILayout.BeginHorizontal();

            // Name
            GUILayout.Label(name, EditorStyles.boldLabel, GUILayout.Width(140));

            // Status Badge
            var prevBg = GUI.backgroundColor;
            GUI.backgroundColor = isRunning ? new Color(0.3f, 0.9f, 0.4f) : new Color(0.9f, 0.4f, 0.4f);
            GUILayout.Box(isRunning ? "RUNNING" : "STOPPED", GUILayout.Width(75), GUILayout.Height(18));
            GUI.backgroundColor = prevBg;

            GUILayout.Label(statusText, EditorStyles.miniLabel, GUILayout.Width(110));

            // Ports
            GUILayout.Label(portsText, EditorStyles.miniLabel, GUILayout.ExpandWidth(true));

            // Actions
            if (onPrimaryAction != null)
            {
                if (GUILayout.Button("Restart", EditorStyles.miniButtonLeft, GUILayout.Width(55), GUILayout.Height(18)))
                    onPrimaryAction();
            }
            if (onSecondaryAction != null)
            {
                if (GUILayout.Button(name.Contains("ros2") ? "Topics" : "Logs", EditorStyles.miniButtonRight, GUILayout.Width(55), GUILayout.Height(18)))
                    onSecondaryAction();
            }

            EditorGUILayout.EndHorizontal();
            EditorGUILayout.Space(1);
        }

        private void DrawContainerLifecycleSection()
        {
            _foldLifecycle = EditorGUILayout.Foldout(_foldLifecycle, "1. Airframe Model Selection & Container Lifecycle", true, EditorStyles.foldoutHeader);
            if (!_foldLifecycle) return;

            EditorGUILayout.BeginVertical(EditorStyles.helpBox);

            // Airframe presets
            EditorGUILayout.LabelField("Quick Start with Airframe Model:", EditorStyles.boldLabel);
            EditorGUILayout.BeginHorizontal();

            GUI.backgroundColor = _airframeModel == "iris" ? new Color(0.7f, 1f, 0.7f) : Color.white;
            if (GUILayout.Button("Quadcopter (iris)", GUILayout.Height(30)))
            {
                _airframeModel = "iris";
                StartOrSwitchAirframe("iris", VehicleAirframeCategory.Multirotor);
            }

            GUI.backgroundColor = _airframeModel == "plane" ? new Color(0.7f, 1f, 0.7f) : Color.white;
            if (GUILayout.Button("Fixed-Wing (plane)", GUILayout.Height(30)))
            {
                _airframeModel = "plane";
                StartOrSwitchAirframe("plane", VehicleAirframeCategory.FixedWing);
            }

            GUI.backgroundColor = _airframeModel == "standard_vtol" ? new Color(0.7f, 1f, 0.7f) : Color.white;
            if (GUILayout.Button("QuadPlane (standard_vtol)", GUILayout.Height(30)))
            {
                _airframeModel = "standard_vtol";
                StartOrSwitchAirframe("standard_vtol", VehicleAirframeCategory.HybridQuadPlaneVtol);
            }
            GUI.backgroundColor = Color.white;

            EditorGUILayout.EndHorizontal();
            EditorGUILayout.Space(4);

            // Container Actions
            EditorGUILayout.BeginHorizontal();
            if (GUILayout.Button("Restart Containers", GUILayout.Height(24)))
            {
                RunScriptAction($"restart {_airframeModel}", $"Restarting containers with model '{_airframeModel}'...");
            }

            GUI.backgroundColor = new Color(1f, 0.45f, 0.45f);
            if (GUILayout.Button("Stop Containers (Keep VM Warm)", GUILayout.Height(24)))
            {
                RunScriptAction("down", "Stopping unity-px4-sitl and unity-ros2-dds containers...");
            }
            GUI.backgroundColor = Color.white;

            if (GUILayout.Button("Open Logs in Terminal", GUILayout.Height(24)))
            {
                OpenLogsInTerminal();
            }
            EditorGUILayout.EndHorizontal();

            EditorGUILayout.EndVertical();
        }

        private void DrawColimaVmAndRuntimeSection()
        {
            _foldVmRuntime = EditorGUILayout.Foldout(_foldVmRuntime, "2. Colima VM & containerd Runtime Service", true, EditorStyles.foldoutHeader);
            if (!_foldVmRuntime) return;

            EditorGUILayout.BeginVertical(EditorStyles.helpBox);
            EditorGUILayout.LabelField("Manage the underlying macOS Apple Virtualization.framework VM & systemd containerd daemon.", EditorStyles.miniLabel);

            EditorGUILayout.BeginHorizontal();
            if (GUILayout.Button("Check VM & containerd Health", GUILayout.Height(26)))
            {
                RunScriptAction("vm-status", "Querying Colima VM & containerd status...");
            }

            GUI.backgroundColor = new Color(0.4f, 1f, 0.7f);
            if (GUILayout.Button("Repair containerd Service", GUILayout.Height(26)))
            {
                RunScriptAction("vm-repair", "Repairing and restarting containerd inside Colima VM...");
            }
            GUI.backgroundColor = Color.white;
            EditorGUILayout.EndHorizontal();

            EditorGUILayout.BeginHorizontal();
            if (GUILayout.Button("Start Colima VM (flightdynamics)", GUILayout.Height(24)))
            {
                RunScriptAction("vm-start", "Starting / verifying Colima VM...");
            }

            GUI.backgroundColor = new Color(1f, 0.45f, 0.45f);
            if (GUILayout.Button("Shutdown Colima VM (Reclaim RAM/CPU)", GUILayout.Height(24)))
            {
                RunScriptAction("vm-stop", "Completely stopping Colima VM profile (flightdynamics)...");
            }
            GUI.backgroundColor = Color.white;
            EditorGUILayout.EndHorizontal();

            EditorGUILayout.EndVertical();
        }

        private void DrawPxhCommandSection()
        {
            _foldPxh = EditorGUILayout.Foldout(_foldPxh, "3. PX4 NuttX/POSIX Shell (pxh) Interactive Commands", true, EditorStyles.foldoutHeader);
            if (!_foldPxh) return;

            EditorGUILayout.BeginVertical(EditorStyles.helpBox);

            // Flight Actions
            EditorGUILayout.BeginHorizontal();
            if (GUILayout.Button("Arm")) SendPxh("commander arm");
            if (GUILayout.Button("Disarm")) SendPxh("commander disarm");
            if (GUILayout.Button("Takeoff")) SendPxh("commander takeoff");
            if (GUILayout.Button("Land")) SendPxh("commander land");
            if (GUILayout.Button("Loiter")) SendPxh("commander mode auto:loiter");
            if (GUILayout.Button("RTL")) SendPxh("commander mode auto:rtl");
            EditorGUILayout.EndHorizontal();

            // VTOL Transitions
            EditorGUILayout.BeginHorizontal();
            GUI.backgroundColor = new Color(0.6f, 0.85f, 1f);
            if (GUILayout.Button("VTOL -> Fixed-Wing Transition"))
                SendPxh("commander vtol_transition fixedwing");
            if (GUILayout.Button("VTOL -> Multicopter Transition"))
                SendPxh("commander vtol_transition multicopter");
            GUI.backgroundColor = Color.white;
            EditorGUILayout.EndHorizontal();

            // Diagnostics
            EditorGUILayout.BeginHorizontal();
            if (GUILayout.Button("Preflight Check")) SendPxh("commander check");
            if (GUILayout.Button("MAVLink Status")) SendPxh("mavlink status");
            if (GUILayout.Button("uORB Top")) SendPxh("top -1");
            EditorGUILayout.EndHorizontal();

            EditorGUILayout.Space(2);

            // Custom Input
            EditorGUILayout.BeginHorizontal();
            GUILayout.Label("pxh>", GUILayout.Width(35));
            _customPxhCommand = EditorGUILayout.TextField(_customPxhCommand);
            if (GUILayout.Button("Send", GUILayout.Width(65)))
            {
                if (!string.IsNullOrWhiteSpace(_customPxhCommand))
                    SendPxh(_customPxhCommand.Trim());
            }
            EditorGUILayout.EndHorizontal();

            EditorGUILayout.EndVertical();
        }

        private void DrawRos2Section()
        {
            _foldRos2 = EditorGUILayout.Foldout(_foldRos2, "4. ROS 2 Humble & MicroXRCE-DDS Control", true, EditorStyles.foldoutHeader);
            if (!_foldRos2) return;

            EditorGUILayout.BeginVertical(EditorStyles.helpBox);
            EditorGUILayout.LabelField("Control the 'unity-ros2-dds' container (MicroXRCEAgent on UDP :8888)", EditorStyles.miniLabel);

            EditorGUILayout.BeginHorizontal();
            if (GUILayout.Button("List Active ROS 2 Topics", GUILayout.Height(24)))
            {
                RunScriptAction("ros2-topics", "Listing active ROS 2 topics in unity-ros2-dds...");
            }
            EditorGUILayout.EndHorizontal();

            EditorGUILayout.BeginHorizontal();
            _offboardAltitude = EditorGUILayout.FloatField("Orbit Alt (m):", _offboardAltitude);
            _offboardRadius = EditorGUILayout.FloatField("Radius (m):", _offboardRadius);
            EditorGUILayout.EndHorizontal();

            EditorGUILayout.BeginHorizontal();
            if (GUILayout.Button("Start Offboard Node", GUILayout.Height(24)))
            {
                var launcher = FindFirstObjectByType<Px4SitlProcessLauncher>();
                if (launcher != null)
                {
                    launcher.StartRos2OffboardNode(_offboardAltitude, _offboardRadius);
                }
                else
                {
                    RunScriptAction($"ros2-offboard --altitude {_offboardAltitude:F1} --radius {_offboardRadius:F1}", "Starting ROS 2 Offboard Node...");
                }
            }

            if (GUILayout.Button("Stop Offboard Node", GUILayout.Height(24)))
            {
                var launcher = FindFirstObjectByType<Px4SitlProcessLauncher>();
                if (launcher != null)
                {
                    launcher.StopRos2OffboardNode();
                }
            }
            EditorGUILayout.EndHorizontal();

            EditorGUILayout.EndVertical();
        }

        private void DrawLiveConsoleSection()
        {
            _foldConsole = EditorGUILayout.Foldout(_foldConsole, "5. Live Container & Runtime Service Console Log", true, EditorStyles.foldoutHeader);
            if (!_foldConsole) return;

            EditorGUILayout.BeginVertical(EditorStyles.helpBox);

            EditorGUILayout.BeginHorizontal();
            _autoScrollConsole = EditorGUILayout.ToggleLeft("Auto Scroll", _autoScrollConsole, GUILayout.Width(100));
            GUILayout.FlexibleSpace();
            if (GUILayout.Button("Clear Console", GUILayout.Width(95), GUILayout.Height(18)))
            {
                _consoleLogBuffer.Clear();
            }
            if (GUILayout.Button("Copy All", GUILayout.Width(75), GUILayout.Height(18)))
            {
                EditorGUIUtility.systemCopyBuffer = _consoleLogBuffer.ToString();
            }
            EditorGUILayout.EndHorizontal();

            var textStyle = new GUIStyle(EditorStyles.textArea)
            {
                fontSize = 11,
                wordWrap = true,
                normal = { textColor = new Color(0.85f, 0.95f, 0.85f), background = Texture2D.blackTexture }
            };

            _consoleScrollPos = EditorGUILayout.BeginScrollView(_consoleScrollPos, GUILayout.Height(180));
            EditorGUILayout.TextArea(_consoleLogBuffer.ToString(), textStyle, GUILayout.ExpandHeight(true));
            if (_autoScrollConsole && Event.current.type == EventType.Repaint)
            {
                _consoleScrollPos.y = float.MaxValue;
            }
            EditorGUILayout.EndScrollView();

            EditorGUILayout.EndVertical();
        }

        private void StartOrSwitchAirframe(string model, VehicleAirframeCategory category)
        {
            _airframeModel = model;
            var launcher = FindFirstObjectByType<Px4SitlProcessLauncher>();
            if (launcher != null)
            {
                launcher.Px4AirframeModel = model;
                launcher.RuntimeBackend = _backend;
                launcher.SwitchAirframeModelAndRestart(model);
            }
            else
            {
                RunScriptAction($"up {model}", $"Starting container stack with model '{model}'...");
            }

            var simMgr = FindFirstObjectByType<FlightSimulationManager>();
            if (simMgr != null)
            {
                simMgr.SwitchVehiclePresetLive(category);
            }

            TriggerAsyncStatusQuery();
        }

        private void SendPxh(string command)
        {
            var launcher = FindFirstObjectByType<Px4SitlProcessLauncher>();
            if (launcher != null)
            {
                launcher.SendPxhCommand(command);
            }
            else
            {
                RunScriptAction($"pxh {command}", $"[pxh>] {command}");
            }
        }

        private void RunScriptAction(string scriptArgs, string summaryMessage)
        {
            string scriptPath = GetManagerScriptPath();
            string wsRoot = GetWorkspaceRoot();

            if (!File.Exists(scriptPath))
            {
                _pendingLogLines.Enqueue($"[Error] Could not find script at {scriptPath}");
                return;
            }

            _lastStatusSummary = summaryMessage;
            _pendingLogLines.Enqueue($"\n>>> {summaryMessage}");
            _isOperationRunning = true;

            try
            {
                string envPrefix = GetBackendEnvPrefix();
                var psi = new ProcessStartInfo
                {
                    FileName = "/bin/bash",
                    Arguments = $"-lc \"{envPrefix}'{scriptPath}' {scriptArgs}\"",
                    WorkingDirectory = wsRoot,
                    UseShellExecute = false,
                    RedirectStandardOutput = true,
                    RedirectStandardError = true,
                    CreateNoWindow = true
                };

                var proc = Process.Start(psi);
                if (proc != null)
                {
                    proc.EnableRaisingEvents = true;
                    proc.Exited += (_, _) =>
                    {
                        _isOperationRunning = false;
                        TriggerAsyncStatusQuery();
                    };
                    proc.OutputDataReceived += (_, e) =>
                    {
                        if (!string.IsNullOrEmpty(e.Data)) _pendingLogLines.Enqueue(e.Data);
                    };
                    proc.ErrorDataReceived += (_, e) =>
                    {
                        if (!string.IsNullOrEmpty(e.Data)) _pendingLogLines.Enqueue("[ERR] " + e.Data);
                    };
                    proc.BeginOutputReadLine();
                    proc.BeginErrorReadLine();
                }
                else
                {
                    _isOperationRunning = false;
                }
            }
            catch (Exception ex)
            {
                _isOperationRunning = false;
                _pendingLogLines.Enqueue($"[Exception] Failed running command '{scriptArgs}': {ex.Message}");
            }
        }

        private void OpenLogsInTerminal()
        {
            try
            {
                string wsRoot = GetWorkspaceRoot();
                string script = $"tell application \"Terminal\" to do script \"cd '{wsRoot}' && ./scripts/px4-sitl.sh logs\"";
                Process.Start("osascript", $"-e \"{script}\"");
            }
            catch (Exception ex)
            {
                Debug.LogWarning($"[FlightDynamics6DoF] Could not open terminal: {ex.Message}");
            }
        }
    }
}
