// Licensed under the Apache License, Version 2.0.
#nullable enable
using System;
using FlightDynamics6DoF.Core.Aerodynamics;
using FlightDynamics6DoF.Core.Specs;
using FlightDynamics6DoF.Unity;
using UnityEditor;
using UnityEngine;

namespace FlightDynamics6DoF.Editor
{
    /// <summary>
    /// Unified 5-Step Custom Editor Workbench coordinator.
    /// Each step UI is encapsulated in its own panel class and infrastructure
    /// process/status state lives in SitlInfraController.
    /// </summary>
    public class FlightDynamicsEngineeringWizard : EditorWindow
    {
        private int _selectedTab = 0;
        private Vector2 _scrollPos;

        internal AircraftProfileAsset? TargetProfile;
        internal FlightDynamicsVehicle? SceneVehicle;
        internal readonly SitlInfraController Infra = new SitlInfraController();
        internal string AirframeModel = "iris";
        internal VehicleAirframeCategory PresetCategory = VehicleAirframeCategory.Multirotor;

        private AircraftConfigSpec? _cachedConfig;
        private BakedAeroDatabase? _cachedDb;

        private readonly AeroDesignPanel _aeroPanel = new AeroDesignPanel();
        private readonly InfraControlPanel _infraPanel = new InfraControlPanel();
        private readonly SimulationEnvironmentPanel _simPanel = new SimulationEnvironmentPanel();
        private readonly Px4TuningPanel _tuningPanel = new Px4TuningPanel();
        private readonly TelemetryLogPanel _telemetryPanel = new TelemetryLogPanel();
        private readonly Ros2Nav2Panel _ros2Panel = new Ros2Nav2Panel();

        private static readonly string[] StepTitles = new[]
        {
            "1. Aero/Design",
            "2. Infra",
            "3. Environment",
            "4. PX4 Tuning",
            "5. Telemetry",
            "6. ROS2/Nav2"
        };

        [MenuItem("Tools/FlightDynamics 6-DOF/Unified 5-Step Engineering Workbench", false, 10)]
        [MenuItem("Window/Aerospace/Unified 5-Step Engineering Workbench", false, 10)]
        public static void ShowWindow() => OpenStep(0);

        public static void OpenStep(int stepIndex)
        {
            var win = GetWindow<FlightDynamicsEngineeringWizard>("FlightDynamics 6-DOF Workbench");
            win.minSize = new Vector2(840, 700);
            win._selectedTab = Mathf.Clamp(stepIndex, 0, StepTitles.Length - 1);
            win.Show();
        }

        internal void SelectTab(int tabIndex) => _selectedTab = Mathf.Clamp(tabIndex, 0, StepTitles.Length - 1);

        internal void SyncEditingParamTableFromVehicle()
        {
            if (SceneVehicle != null)
                _tuningPanel.EditingParamTable = SceneVehicle.ControlParameterTable;
        }

        private void OnEnable()
        {
            RefreshSceneRefs();
            EditorApplication.update += HandleEditorUpdate;
            EditorApplication.playModeStateChanged += HandlePlayModeStateChanged;
            Infra.TriggerStatusQuery(EditorApplication.timeSinceStartup);
        }

        private void OnDisable()
        {
            _ros2Panel.StopNav2();
            EditorApplication.update -= HandleEditorUpdate;
            EditorApplication.playModeStateChanged -= HandlePlayModeStateChanged;
        }

        private void HandlePlayModeStateChanged(PlayModeStateChange state)
        {
            if (state == PlayModeStateChange.EnteredEditMode)
            {
                _telemetryPanel.RefreshSavedFlightLogList(autoLoadLatestIfEmpty: false);
                Repaint();
            }
            else if (state == PlayModeStateChange.ExitingPlayMode && SceneVehicle != null && SceneVehicle.TelemetryLogger.Frames.Count > 1)
            {
                SceneVehicle.AutoSaveTelemetryLogs();
                _telemetryPanel.RefreshSavedFlightLogList(autoLoadLatestIfEmpty: false);
            }
        }

        private void RefreshSceneRefs()
        {
            if (SceneVehicle == null)
                SceneVehicle = FindFirstObjectByType<FlightDynamicsVehicle>();

            if (SceneVehicle != null)
            {
                if (TargetProfile == null && SceneVehicle.ProfileAsset != null)
                    TargetProfile = SceneVehicle.ProfileAsset;

                PresetCategory = SceneVehicle.ActiveConfig.Category;
                AirframeModel = _tuningPanel.SyncTableForCategory(
                    PresetCategory,
                    SceneVehicle.ActiveConfig,
                    SceneVehicle.ActiveBakedDatabase,
                    SceneVehicle.ControlParameterTable);
            }
            else if (TargetProfile != null)
            {
                PresetCategory = TargetProfile.Config.Category;
                AirframeModel = _tuningPanel.SyncTableForCategory(
                    PresetCategory,
                    TargetProfile.Config,
                    TargetProfile.BakedDatabase);
            }
            else
            {
                AirframeModel = _tuningPanel.SyncTableForCategory(
                    PresetCategory,
                    GetCurrentConfig(),
                    GetCurrentBakedDb());
            }
        }

        private void HandleEditorUpdate()
        {
            bool shouldRepaint = Infra.Tick(EditorApplication.timeSinceStartup);

            if (_selectedTab == 4 && EditorApplication.isPlaying &&
                _telemetryPanel.ShouldRepaintOnPlay(EditorApplication.timeSinceStartup))
            {
                shouldRepaint = true;
            }

            if (shouldRepaint)
                Repaint();
        }

        private void OnGUI()
        {
            RefreshSceneRefs();
            DrawPersistentHeader();

            _selectedTab = GUILayout.Toolbar(_selectedTab, StepTitles, GUILayout.Height(30));
            EditorGUILayout.Space(6);

            _scrollPos = EditorGUILayout.BeginScrollView(_scrollPos);
            switch (_selectedTab)
            {
                case 0: _aeroPanel.Draw(this); break;
                case 1: _infraPanel.Draw(Infra, AirframeModel, () => _tuningPanel.ExportParamsToDockerConfig(this)); break;
                case 2: _simPanel.Draw(this); break;
                case 3: _tuningPanel.Draw(this); break;
                case 4: _telemetryPanel.Draw(this); break;
                case 5: _ros2Panel.Draw(this); break;
            }
            EditorGUILayout.EndScrollView();

            DrawStepFooterNav();
        }

        private void DrawPersistentHeader()
        {
            EditorGUILayout.BeginVertical(EditorStyles.helpBox);
            EditorGUILayout.BeginHorizontal();
            GUILayout.Label("FlightDynamics 6-DOF: Unified 6-Step SITL Workbench", EditorStyles.boldLabel);
            GUILayout.FlexibleSpace();

            if (!EditorApplication.isPlaying)
            {
                GUI.backgroundColor = new Color(0.45f, 0.95f, 0.55f);
                if (GUILayout.Button("▶ Start Sim", GUILayout.Width(120), GUILayout.Height(22)))
                    StartSimulation();
            }
            else
            {
                GUI.backgroundColor = new Color(1f, 0.4f, 0.4f);
                if (GUILayout.Button("■ Stop Sim", GUILayout.Width(120), GUILayout.Height(22)))
                    StopSimulation();
            }
            GUI.backgroundColor = Color.white;

            Infra.AutoRefreshStatus = EditorGUILayout.ToggleLeft("Auto Poll", Infra.AutoRefreshStatus, GUILayout.Width(82));
            GUI.enabled = !Infra.IsQueryingStatus;
            if (GUILayout.Button(Infra.IsQueryingStatus ? "Polling..." : "Refresh", GUILayout.Width(72), GUILayout.Height(22)))
                Infra.TriggerStatusQuery(EditorApplication.timeSinceStartup);
            GUI.enabled = true;
            EditorGUILayout.EndHorizontal();

            EditorGUILayout.BeginHorizontal();
            bool hasSitlWs = Infra.IsSitlWorkspaceAvailable;
            bool vmUp = !hasSitlWs || Infra.VmStatus.Equals("RUNNING", StringComparison.OrdinalIgnoreCase);
            bool cdUp = !hasSitlWs || Infra.ContainerdStatus.Equals("ACTIVE", StringComparison.OrdinalIgnoreCase);
            bool px4Up = Infra.IsContainerRunning("unity-px4-sitl");
            bool rosUp = Infra.IsContainerRunning("unity-ros2-dds");

            DrawCompactBadge($"Backend ({Infra.DetectedBackendName})", vmUp ? Infra.VmStatus : Infra.VmStatus, vmUp ? new Color(0.25f, 0.85f, 0.35f) : new Color(0.9f, 0.35f, 0.35f));
            DrawCompactBadge(hasSitlWs ? "containerd" : "Autopilot", cdUp ? Infra.ContainerdStatus : Infra.ContainerdStatus, cdUp ? new Color(0.25f, 0.85f, 0.35f) : new Color(0.95f, 0.65f, 0.25f));
            DrawCompactBadge("unity-px4-sitl", px4Up ? $"UP ({AirframeModel})" : (hasSitlWs ? "STOPPED" : "OPTIONAL (TCP:4560)"), px4Up ? new Color(0.25f, 0.85f, 0.35f) : (hasSitlWs ? new Color(0.85f, 0.4f, 0.4f) : new Color(0.55f, 0.75f, 0.95f)));
            DrawCompactBadge("unity-ros2-dds", rosUp ? "UP (UDP:8888)" : (hasSitlWs ? "STOPPED" : "OPTIONAL (UDP:8888)"), rosUp ? new Color(0.25f, 0.85f, 0.35f) : (hasSitlWs ? new Color(0.85f, 0.4f, 0.4f) : new Color(0.55f, 0.75f, 0.95f)));

            var activeLogger = _telemetryPanel.GetActiveLogger(SceneVehicle);
            string simState = EditorApplication.isPlaying ? $"PLAY ({activeLogger.Frames.Count}f)" : $"IDLE ({activeLogger.Frames.Count}f)";
            DrawCompactBadge("Simulation", simState, EditorApplication.isPlaying ? new Color(0.3f, 0.85f, 1f) : Color.gray);
            EditorGUILayout.EndHorizontal();

            EditorGUILayout.BeginHorizontal();
            TargetProfile = (AircraftProfileAsset?)EditorGUILayout.ObjectField("Profile Asset", TargetProfile, typeof(AircraftProfileAsset), false);
            if (GUILayout.Button("New Asset", GUILayout.Width(80)))
                AeroDesignPanel.CreateNewProfileAsset(this);
            SceneVehicle = (FlightDynamicsVehicle?)EditorGUILayout.ObjectField("Scene Vehicle", SceneVehicle, typeof(FlightDynamicsVehicle), true);
            EditorGUILayout.EndHorizontal();

            EditorGUILayout.EndVertical();
        }

        private static void DrawCompactBadge(string label, string state, Color col)
        {
            EditorGUILayout.BeginVertical(EditorStyles.helpBox, GUILayout.MinWidth(105));
            GUILayout.Label(label, EditorStyles.miniLabel);
            var prev = GUI.contentColor;
            GUI.contentColor = col;
            GUILayout.Label(state, EditorStyles.miniBoldLabel);
            GUI.contentColor = prev;
            EditorGUILayout.EndVertical();
        }

        private void DrawStepFooterNav()
        {
            EditorGUILayout.BeginHorizontal(EditorStyles.helpBox);
            GUI.enabled = _selectedTab > 0;
            if (GUILayout.Button("<< Prev Step", GUILayout.Width(130), GUILayout.Height(24)))
                _selectedTab--;
            GUI.enabled = true;

            GUILayout.FlexibleSpace();
            GUILayout.Label($"Current Step: {StepTitles[_selectedTab]}", EditorStyles.miniBoldLabel);
            GUILayout.FlexibleSpace();

            GUI.enabled = _selectedTab < StepTitles.Length - 1;
            if (GUILayout.Button("Next Step >>", GUILayout.Width(130), GUILayout.Height(24)))
                _selectedTab++;
            GUI.enabled = true;
            EditorGUILayout.EndHorizontal();
        }

        internal void StartSimulation()
        {
            if (SceneVehicle == null)
                BuildCompleteSimulationScene();

            if (SceneVehicle != null)
            {
                SceneVehicle.TelemetryLogger.Clear();
                SceneVehicle.TelemetryLogger.StartRecording();
            }

            EditorApplication.isPlaying = true;
        }

        internal void StopSimulation()
        {
            if (SceneVehicle != null && SceneVehicle.TelemetryLogger.Frames.Count > 1)
                SceneVehicle.AutoSaveTelemetryLogs();

            EditorApplication.isPlaying = false;

            if (_simPanel.StopContainersOnSimExit)
                Infra.RunScriptAction("down", "Stopping PX4 & ROS 2 containers on simulation exit...", () => EditorApplication.timeSinceStartup);
        }

        internal AircraftConfigSpec GetCurrentConfig()
        {
            if (SceneVehicle != null) return SceneVehicle.ActiveConfig;
            if (TargetProfile != null) return TargetProfile.Config;
            if (_cachedConfig == null)
            {
                _cachedConfig = PresetCategory switch
                {
                    VehicleAirframeCategory.FixedWing => AircraftConfigSpec.CreateFixedWingUav(),
                    VehicleAirframeCategory.HybridQuadPlaneVtol => AircraftConfigSpec.CreateQuadPlaneVtolMillie(),
                    _ => AircraftConfigSpec.CreateStandardQuadcopterX()
                };
            }
            return _cachedConfig;
        }

        internal BakedAeroDatabase GetCurrentBakedDb()
        {
            if (SceneVehicle != null) return SceneVehicle.ActiveBakedDatabase;
            if (TargetProfile != null) return TargetProfile.BakedDatabase;
            if (_cachedDb == null)
                _cachedDb = StabilityAndTrimAnalyzer.AnalyzeAndBake(GetCurrentConfig());
            return _cachedDb;
        }

        internal string RebakeActiveConfig(AircraftConfigSpec cfg)
        {
            string targetDesc = "Offline Preset Cache";
            if (TargetProfile != null)
            {
                Undo.RecordObject(TargetProfile, "Modify Aircraft Spec");
                TargetProfile.Config = cfg;
                TargetProfile.AnalyzeAndBake();
                EditorUtility.SetDirty(TargetProfile);
                targetDesc = $"ProfileAsset ({TargetProfile.name})";
            }
            if (SceneVehicle != null)
            {
                Undo.RecordObject(SceneVehicle, "Modify Scene Vehicle Spec");
                Undo.RecordObject(SceneVehicle.transform, "Update Scene Vehicle Ground Height");
                SceneVehicle.InlineConfig = cfg;
                SceneVehicle.RunStage1AnalysisAndBake();
                ProceduralAircraftVisualBuilder.RebuildVehicleVisuals(SceneVehicle);
                if (!Application.isPlaying && SceneVehicle.transform.position.y < 1.5f)
                {
                    var p = SceneVehicle.transform.position;
                    SceneVehicle.transform.position = new Vector3(p.x, Mathf.Max(0.06f, (float)cfg.LandingGearHeightMeters), p.z);
                }
                else if (Application.isPlaying && SceneVehicle.Engine != null && SceneVehicle.Engine.State.IsGrounded)
                {
                    var p = SceneVehicle.transform.position;
                    SceneVehicle.transform.position = new Vector3(p.x, Mathf.Max(0.06f, (float)cfg.LandingGearHeightMeters), p.z);
                    SceneVehicle.InitializeSimulationEngine();
                }
                EditorUtility.SetDirty(SceneVehicle);
                SceneView.RepaintAll();
                targetDesc = TargetProfile != null ? $"{targetDesc} + SceneVehicle" : $"SceneVehicle ({SceneVehicle.name})";
            }
            if (TargetProfile == null && SceneVehicle == null)
            {
                _cachedConfig = cfg;
                _cachedDb = StabilityAndTrimAnalyzer.AnalyzeAndBake(cfg);
            }
            return targetDesc;
        }

        internal void SwitchAirframeEverywhere(VehicleAirframeCategory category, string modelName)
        {
            PresetCategory = category;
            AirframeModel = modelName;

            if (TargetProfile != null)
            {
                Undo.RecordObject(TargetProfile, "Switch Airframe Profile");
                TargetProfile.LoadPreset(category);
                TargetProfile.AnalyzeAndBake();
                EditorUtility.SetDirty(TargetProfile);
            }

            var simMgr = FindFirstObjectByType<FlightSimulationManager>();
            if (simMgr != null)
            {
                simMgr.SwitchVehicleAndPx4Model(category);
                SceneVehicle = simMgr.PrimaryVehicle;
            }
            else if (SceneVehicle != null)
            {
                Undo.RecordObject(SceneVehicle, "Switch Scene Vehicle Airframe");
                SceneVehicle.InlineConfig = category switch
                {
                    VehicleAirframeCategory.FixedWing => AircraftConfigSpec.CreateFixedWingUav(),
                    VehicleAirframeCategory.HybridQuadPlaneVtol => AircraftConfigSpec.CreateQuadPlaneVtolMillie(),
                    _ => AircraftConfigSpec.CreateStandardQuadcopterX()
                };
                SceneVehicle.RunStage1AnalysisAndBake();
                EditorUtility.SetDirty(SceneVehicle);
            }
            else
            {
                _cachedConfig = category switch
                {
                    VehicleAirframeCategory.FixedWing => AircraftConfigSpec.CreateFixedWingUav(),
                    VehicleAirframeCategory.HybridQuadPlaneVtol => AircraftConfigSpec.CreateQuadPlaneVtolMillie(),
                    _ => AircraftConfigSpec.CreateStandardQuadcopterX()
                };
                _cachedDb = StabilityAndTrimAnalyzer.AnalyzeAndBake(_cachedConfig);
            }

            var cfg = GetCurrentConfig();
            var db = GetCurrentBakedDb();
            _tuningPanel.ResetTableForCategory(category, cfg, db, SceneVehicle);
            AirframeModel = _tuningPanel.EditingParamTable?.Px4SitlModelName ?? modelName;
            _tuningPanel.ExportParamsToDockerConfig(this);
        }

        internal void BuildCompleteSimulationScene()
        {
            var mgr = FlightSimulationManager.SpawnCompleteSimulationEnvironment(PresetCategory, _simPanel.Fidelity, _simPanel.MavlinkMode, _simPanel.AutoTakeoff);
            SceneVehicle = mgr.PrimaryVehicle;
            if (SceneVehicle != null)
            {
                _tuningPanel.EditingParamTable = SceneVehicle.ControlParameterTable;
                AirframeModel = SceneVehicle.ControlParameterTable.Px4SitlModelName;
                Selection.activeGameObject = SceneVehicle.gameObject;
                if (SceneView.lastActiveSceneView != null)
                    SceneView.lastActiveSceneView.FrameSelected();
            }
        }
    }
}
