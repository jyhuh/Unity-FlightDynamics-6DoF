// Licensed under the Apache License, Version 2.0.
#nullable enable
using System.Diagnostics;
using UnityEditor;
using UnityEngine;
using FlightDynamics6DoF.Core.Specs;
using FlightDynamics6DoF.Unity;
using Debug = UnityEngine.Debug;

namespace FlightDynamics6DoF.Editor
{
    /// <summary>
    /// Custom Inspector Editor for <see cref="Px4SitlProcessLauncher"/>.
    /// Eliminates all need for external terminal/CLI operations:
    /// provides one-click container management, model switching (iris, plane, standard_vtol),
    /// interactive pxh command execution, and live container log inspection directly within the Unity Editor.
    /// </summary>
    [CustomEditor(typeof(Px4SitlProcessLauncher))]
    public class Px4SitlProcessLauncherEditor : UnityEditor.Editor
    {
        private string _customCommand = "commander takeoff";
        private bool _showPxhFold = true;
        private bool _showContainerFold = true;

        public override void OnInspectorGUI()
        {
            var launcher = (Px4SitlProcessLauncher)target;

            EditorGUILayout.BeginVertical(EditorStyles.helpBox);
            GUILayout.Label("PX4-Autopilot SITL & ROS 2 Container Stack Control", EditorStyles.boldLabel);
            EditorGUILayout.LabelField("Status", launcher.IsPx4ProcessRunning ? "RUNNING (Containers Active)" : "STOPPED / STANDBY");
            EditorGUILayout.LabelField("Active Airframe", launcher.Px4AirframeModel);
            EditorGUILayout.LabelField("Console Output", launcher.LastConsoleSummary, EditorStyles.wordWrappedMiniLabel);
            EditorGUILayout.EndVertical();

            EditorGUILayout.Space(4);

            // 1. One-Click Container Launch / Airframe Switch
            _showContainerFold = EditorGUILayout.Foldout(_showContainerFold, "1. Airframe Selection & Container Lifecycle", true, EditorStyles.foldoutHeader);
            if (_showContainerFold)
            {
                EditorGUILayout.BeginVertical(EditorStyles.helpBox);
                EditorGUILayout.BeginHorizontal();
                if (GUILayout.Button("Start Quadcopter (iris)", GUILayout.Height(28)))
                {
                    launcher.SwitchAirframeModelAndRestart("iris");
                    SyncVehicleModelIfExists(launcher, VehicleAirframeCategory.Multirotor);
                }
                if (GUILayout.Button("Start Fixed-Wing (plane)", GUILayout.Height(28)))
                {
                    launcher.SwitchAirframeModelAndRestart("plane");
                    SyncVehicleModelIfExists(launcher, VehicleAirframeCategory.FixedWing);
                }
                if (GUILayout.Button("Start QuadPlane (standard_vtol)", GUILayout.Height(28)))
                {
                    launcher.SwitchAirframeModelAndRestart("standard_vtol");
                    SyncVehicleModelIfExists(launcher, VehicleAirframeCategory.HybridQuadPlaneVtol);
                }
                EditorGUILayout.EndHorizontal();

                EditorGUILayout.BeginHorizontal();
                GUI.backgroundColor = new Color(1f, 0.4f, 0.4f);
                if (GUILayout.Button("Stop Containers (Keep VM warm)", GUILayout.Height(24)))
                {
                    launcher.StopPx4SitlProcess();
                }
                if (GUILayout.Button("Shutdown Colima VM (flightdynamics)", GUILayout.Height(24)))
                {
                    launcher.ShutdownColimaVm();
                }
                GUI.backgroundColor = Color.white;
                EditorGUILayout.EndHorizontal();

                EditorGUILayout.BeginHorizontal();
                if (GUILayout.Button("Open Live Container Logs in Terminal", GUILayout.Height(24)))
                {
                    OpenLogsInTerminal(launcher.WorkspaceRootPath);
                }
                EditorGUILayout.EndHorizontal();
                EditorGUILayout.EndVertical();
            }

            EditorGUILayout.Space(4);

            // 2. Interactive pxh Command Console
            _showPxhFold = EditorGUILayout.Foldout(_showPxhFold, "2. PX4 Shell (pxh) Quick Actions & Custom Command", true, EditorStyles.foldoutHeader);
            if (_showPxhFold)
            {
                EditorGUILayout.BeginVertical(EditorStyles.helpBox);

                // Flight Mode & Arming Shortcuts
                EditorGUILayout.BeginHorizontal();
                if (GUILayout.Button("Arm")) launcher.SendPxhCommand("commander arm");
                if (GUILayout.Button("Disarm")) launcher.SendPxhCommand("commander disarm");
                if (GUILayout.Button("Takeoff")) launcher.SendPxhCommand("commander takeoff");
                if (GUILayout.Button("Land")) launcher.SendPxhCommand("commander land");
                if (GUILayout.Button("Loiter")) launcher.SendPxhCommand("commander mode auto:loiter");
                if (GUILayout.Button("RTL")) launcher.SendPxhCommand("commander mode auto:rtl");
                EditorGUILayout.EndHorizontal();

                // VTOL Specific Controls
                EditorGUILayout.BeginHorizontal();
                GUI.backgroundColor = new Color(0.6f, 0.85f, 1f);
                if (GUILayout.Button("VTOL -> Fixed-Wing Transition"))
                    launcher.SendPxhCommand("commander vtol_transition fixedwing");
                if (GUILayout.Button("VTOL -> Multicopter Transition"))
                    launcher.SendPxhCommand("commander vtol_transition multicopter");
                GUI.backgroundColor = Color.white;
                EditorGUILayout.EndHorizontal();

                // Diagnostics
                EditorGUILayout.BeginHorizontal();
                if (GUILayout.Button("Check Preflight")) launcher.SendPxhCommand("commander check");
                if (GUILayout.Button("MAVLink Status")) launcher.SendPxhCommand("mavlink status");
                if (GUILayout.Button("uORB Top")) launcher.SendPxhCommand("top -1");
                EditorGUILayout.EndHorizontal();

                EditorGUILayout.Space(2);

                // Interactive Custom pxh Input Field
                EditorGUILayout.BeginHorizontal();
                GUILayout.Label("pxh>", GUILayout.Width(35));
                _customCommand = EditorGUILayout.TextField(_customCommand);
                if (GUILayout.Button("Send", GUILayout.Width(65)))
                {
                    if (!string.IsNullOrWhiteSpace(_customCommand))
                        launcher.SendPxhCommand(_customCommand.Trim());
                }
                EditorGUILayout.EndHorizontal();

                EditorGUILayout.EndVertical();
            }

            EditorGUILayout.Space(4);

            // 3. ROS 2 Humble Offboard Actions
            EditorGUILayout.BeginVertical(EditorStyles.helpBox);
            GUILayout.Label("3. ROS 2 Humble Offboard Node", EditorStyles.boldLabel);
            EditorGUILayout.BeginHorizontal();
            if (GUILayout.Button("Start Offboard Orbit Node (10m Alt, 8m R)"))
                launcher.StartRos2OffboardNode(10.0f, 8.0f);
            if (GUILayout.Button("Stop Offboard Node"))
                launcher.StopRos2OffboardNode();
            EditorGUILayout.EndHorizontal();
            EditorGUILayout.EndVertical();

            EditorGUILayout.Space(6);
            DrawDefaultInspector();
        }

        private static void SyncVehicleModelIfExists(Px4SitlProcessLauncher launcher, VehicleAirframeCategory category)
        {
            var mgr = launcher.GetComponent<FlightSimulationManager>() ?? FindFirstObjectByType<FlightSimulationManager>();
            if (mgr != null)
            {
                mgr.SwitchVehiclePresetLive(category);
            }
            else if (launcher.TargetVehicle != null)
            {
                launcher.TargetVehicle.InlineConfig = category switch
                {
                    VehicleAirframeCategory.FixedWing => Core.Specs.AircraftConfigSpec.CreateFixedWingUav(),
                    VehicleAirframeCategory.HybridQuadPlaneVtol => Core.Specs.AircraftConfigSpec.CreateQuadPlaneVtolMillie(),
                    _ => Core.Specs.AircraftConfigSpec.CreateStandardQuadcopterX()
                };
                launcher.TargetVehicle.RunStage1AnalysisAndBake();
                ProceduralAircraftVisualBuilder.RebuildVehicleVisuals(launcher.TargetVehicle);
            }
        }

        private static void OpenLogsInTerminal(string workspaceRoot)
        {
            try
            {
                string script = $"tell application \"Terminal\" to do script \"cd '{workspaceRoot}' && ./scripts/px4-sitl.sh logs\"";
                Process.Start("osascript", $"-e \"{script}\"");
            }
            catch (System.Exception ex)
            {
                Debug.LogWarning($"[FlightDynamics6DoF] Could not open terminal logs: {ex.Message}");
            }
        }
    }
}
