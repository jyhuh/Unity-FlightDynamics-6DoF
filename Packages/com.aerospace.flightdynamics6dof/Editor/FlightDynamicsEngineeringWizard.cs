// Licensed under the Apache License, Version 2.0.
#nullable enable
using System;
using System.IO;
using UnityEditor;
using UnityEngine;
using FlightDynamics6DoF.Core.Aerodynamics;
using FlightDynamics6DoF.Core.Gnc;
using FlightDynamics6DoF.Core.Math;
using FlightDynamics6DoF.Core.Mavlink;
using FlightDynamics6DoF.Core.Specs;
using FlightDynamics6DoF.Unity;

namespace FlightDynamics6DoF.Editor
{
    /// <summary>
    /// All-in-one Unity Editor Engineering Workbench, Aerodynamic Calculator (Modules 1-5),
    /// Multi-Fidelity Stage 1/Stage 2 Configurator, PX4 Airframe Exporter, and Scene Setup Wizard.
    /// </summary>
    public class FlightDynamicsEngineeringWizard : EditorWindow
    {
        private int _selectedTab = 0;
        private Vector2 _scrollPos;

        private AircraftProfileAsset? _targetProfile;
        private FlightDynamicsVehicle? _sceneVehicle;

        // Interactive Module 1 Calculator inputs
        private string _nacaCode = "2412";
        private float _testAlphaDeg = 5.0f;
        private float _testReynolds = 500000f;
        private float _testMach = 0.08f;
        private AirfoilPolarPoint _lastAirfoilPoint;
        private bool _hasAirfoilResult;

        // Interactive Module 4 CCBlade BEMT Calculator inputs
        private float _testRotorRpm = 6500f;
        private float _testRotorAxialSpeedMps = 0.0f;
        private float _testRotorAglMeters = 10.0f;
        private RotorBemtSolution _lastRotorSolution;
        private bool _hasRotorResult;

        // Scene setup options
        private VehicleAirframeCategory _wizardPresetCategory = VehicleAirframeCategory.Multirotor;
        private SimulationFidelityLevel _wizardFidelity = SimulationFidelityLevel.Level2_6DoFBladeAndStrip;
        private MavlinkBridgeMode _wizardMavlinkMode = MavlinkBridgeMode.ExternalSitlHil;
        private bool _wizardAutoTakeoff = false;

        private static readonly string[] TabTitles = new[]
        {
            "1. Stage-1 Aero & Stability Analyzer",
            "2. Stage-2 Multi-Fidelity 6-DOF",
            "3. GNC, MAVLink & PX4 Exporter",
            "4. One-Click Scene Setup Wizard"
        };

        [MenuItem("Tools/FlightDynamics 6-DOF/Engineering Workbench & Setup Wizard", false, 10)]
        public static void ShowWindow()
        {
            var win = GetWindow<FlightDynamicsEngineeringWizard>("FlightDynamics 6-DOF Workbench");
            win.minSize = new Vector2(680, 620);
            win.Show();
        }

        private void OnEnable()
        {
            if (_sceneVehicle == null)
                _sceneVehicle = FindFirstObjectByType<FlightDynamicsVehicle>();
        }

        private void OnGUI()
        {
            DrawBannerHeader();

            _selectedTab = GUILayout.Toolbar(_selectedTab, TabTitles, GUILayout.Height(28));
            EditorGUILayout.Space(6);

            _scrollPos = EditorGUILayout.BeginScrollView(_scrollPos);
            switch (_selectedTab)
            {
                case 0:
                    DrawTabStage1AeroAndStabilityAnalyzer();
                    break;
                case 1:
                    DrawTabStage2MultiFidelityConfig();
                    break;
                case 2:
                    DrawTabGncMavlinkAndPx4Export();
                    break;
                case 3:
                    DrawTabOneClickSceneSetup();
                    break;
            }
            EditorGUILayout.EndScrollView();
        }

        private void DrawBannerHeader()
        {
            EditorGUILayout.BeginVertical(EditorStyles.helpBox);
            GUILayout.Label("FlightDynamics 6-DOF: Nonlinear UAV/VTOL FDM & Aerodynamic Engineering Suite", EditorStyles.boldLabel);
            GUILayout.Label("Apache-2.0 Compliant | Stage-1 Pre-Flight Aero/Stability Analyzer (CST/NeuralFoil + Viterna + 3D VLM + CCBlade BEMT) & Stage-2 Multi-Fidelity (Level 0..3) 6-DOF Simulator", EditorStyles.miniLabel);
            EditorGUILayout.EndVertical();

            EditorGUILayout.BeginHorizontal();
            _targetProfile = (AircraftProfileAsset?)EditorGUILayout.ObjectField("Aircraft Profile Asset", _targetProfile, typeof(AircraftProfileAsset), false);
            if (GUILayout.Button("Create New Asset", GUILayout.Width(130)))
            {
                CreateNewProfileAsset();
            }
            EditorGUILayout.EndHorizontal();

            _sceneVehicle = (FlightDynamicsVehicle?)EditorGUILayout.ObjectField("Active Scene Vehicle", _sceneVehicle, typeof(FlightDynamicsVehicle), true);
            EditorGUILayout.Space(4);
        }

        private AircraftConfigSpec GetCurrentConfig()
        {
            if (_targetProfile != null) return _targetProfile.Config;
            if (_sceneVehicle != null) return _sceneVehicle.ActiveConfig;
            return AircraftConfigSpec.CreateStandardQuadcopterX();
        }

        private BakedAeroDatabase GetCurrentBakedDb()
        {
            if (_targetProfile != null) return _targetProfile.BakedDatabase;
            if (_sceneVehicle != null) return _sceneVehicle.ActiveBakedDatabase;
            return StabilityAndTrimAnalyzer.AnalyzeAndBake(GetCurrentConfig());
        }

        private void DrawTabStage1AeroAndStabilityAnalyzer()
        {
            var cfg = GetCurrentConfig();
            var db = GetCurrentBakedDb();

            EditorGUILayout.BeginVertical(EditorStyles.helpBox);
            GUILayout.Label("A. Load Engineering Reference Airframe or Extract 3D Mesh Geometry", EditorStyles.boldLabel);
            EditorGUILayout.BeginHorizontal();
            if (GUILayout.Button("Load Quadcopter X-450")) ApplyPreset(VehicleAirframeCategory.Multirotor);
            if (GUILayout.Button("Load Fixed-Wing Talon-180")) ApplyPreset(VehicleAirframeCategory.FixedWing);
            if (GUILayout.Button("Load QuadPlane VTOL (Millie-Class)")) ApplyPreset(VehicleAirframeCategory.HybridQuadPlaneVtol);
            EditorGUILayout.EndHorizontal();

            if (_sceneVehicle != null && GUILayout.Button("Extract Mass/Inertia & Projected Areas from Scene Vehicle 3D Meshes"))
            {
                Undo.RecordObject(_sceneVehicle, "Extract 3D Mesh Geometry");
                _sceneVehicle.ExtractGeometryFromChildMeshesAndBake();
                EditorUtility.SetDirty(_sceneVehicle);
            }
            EditorGUILayout.EndVertical();

            EditorGUILayout.Space(6);

            // Module 1 Calculator
            EditorGUILayout.BeginVertical(EditorStyles.helpBox);
            GUILayout.Label("B. Module 1: 2D Airfoil Viscous Polar & Viterna (+/-180 deg) Calculator", EditorStyles.boldLabel);
            _nacaCode = EditorGUILayout.TextField("NACA 4-Digit Code", _nacaCode);
            _testAlphaDeg = EditorGUILayout.Slider("Angle of Attack Alpha [deg]", _testAlphaDeg, -180f, 180f);
            _testReynolds = EditorGUILayout.Slider("Reynolds Number (Re)", _testReynolds, 1.0e4f, 1.0e7f);
            _testMach = EditorGUILayout.Slider("Mach Number (M < 0.7)", _testMach, 0.0f, 0.68f);

            if (GUILayout.Button("Evaluate 2D Airfoil Section (CST + NeuralFoil + Viterna)"))
            {
                var af = AirfoilSpec.FromNaca4Digit(_nacaCode, _testReynolds);
                _lastAirfoilPoint = AirfoilPolarSolver.Evaluate(af, _testAlphaDeg * FrameConverter.Deg2Rad, _testReynolds, _testMach);
                _hasAirfoilResult = true;
            }

            if (_hasAirfoilResult)
            {
                EditorGUILayout.HelpBox(
                    $"Alpha = {_lastAirfoilPoint.AlphaDeg:F1} deg | Cl = {_lastAirfoilPoint.Cl:F4} | Cd = {_lastAirfoilPoint.Cd:F4} | Cm = {_lastAirfoilPoint.Cm:F4} | L/D = {(_lastAirfoilPoint.Cl / System.Math.Max(1e-4, _lastAirfoilPoint.Cd)):F1}\n" +
                    $"Boundary-Layer Transition: Top Xtr/c = {_lastAirfoilPoint.TopTransitionXtr:F3} | Bottom Xtr/c = {_lastAirfoilPoint.BotTransitionXtr:F3}",
                    MessageType.Info);
            }
            EditorGUILayout.EndVertical();

            EditorGUILayout.Space(6);

            // Module 4 Calculator
            EditorGUILayout.BeginVertical(EditorStyles.helpBox);
            GUILayout.Label("C. Module 4: CCBlade BEMT Rotor & Slipstream Performance Calculator", EditorStyles.boldLabel);
            _testRotorRpm = EditorGUILayout.Slider("Rotor Speed [RPM]", _testRotorRpm, 500f, 12000f);
            _testRotorAxialSpeedMps = EditorGUILayout.Slider("Axial Inflow Speed V_inf [m/s]", _testRotorAxialSpeedMps, -5f, 35f);
            _testRotorAglMeters = EditorGUILayout.Slider("Altitude AGL (Ground Effect) [m]", _testRotorAglMeters, 0.1f, 20f);

            if (GUILayout.Button("Evaluate Rotor BEMT (Prandtl Tip/Hub Loss + Glauert/Buhl)"))
            {
                RotorSpec rSpec = cfg.Rotors.Count > 0 ? cfg.Rotors[0] : AircraftConfigSpec.CreateStandardQuadcopterX().Rotors[0];
                var atm = IsaAtmosphereAndTurbulence.Evaluate(cfg.DesignAltitudeMsl);
                Vec3d vInf = rSpec.ThrustAxisFrd.Normalized() * _testRotorAxialSpeedMps;
                _lastRotorSolution = CcBladeBemtSolver.EvaluateRotor(rSpec, vInf, Vec3d.Zero, _testRotorRpm, atm, _testRotorAglMeters, true, true);
                _hasRotorResult = true;
            }

            if (_hasRotorResult)
            {
                EditorGUILayout.HelpBox(
                    $"Thrust = {_lastRotorSolution.ThrustNewtons:F2} N | Torque = {_lastRotorSolution.AerodynamicTorqueNm:F4} N*m | Shaft Power = {_lastRotorSolution.ShaftPowerWatts:F1} W\n" +
                    $"J = {_lastRotorSolution.AdvanceRatioJ:F3} | CT = {_lastRotorSolution.Ct:F4} | CQ = {_lastRotorSolution.Cq:F5} | Efficiency Eta = {(_lastRotorSolution.EfficiencyEta * 100.0):F1}% | Mean Induced Vi = {_lastRotorSolution.MeanAxialInducedVelocityMps:F2} m/s",
                    MessageType.Info);
            }
            EditorGUILayout.EndVertical();

            EditorGUILayout.Space(6);

            // Module 2, 3, 5 Full Aircraft Analysis & Bake
            EditorGUILayout.BeginVertical(EditorStyles.helpBox);
            GUILayout.Label("D. Modules 2, 3 & 5: Full-Aircraft 3D VLM, Slender Body, Stability Derivatives & Trim Bake", EditorStyles.boldLabel);
            if (GUILayout.Button("Execute Stage 1 Full Aircraft Analysis & Bake Database", GUILayout.Height(32)))
            {
                RunBakeOnActiveTarget();
                db = GetCurrentBakedDb();
            }

            if (db != null && !string.IsNullOrEmpty(db.AircraftName))
            {
                EditorGUILayout.LabelField("Analyzed Aircraft", $"{db.AircraftName} (Mass: {db.TotalMassKg:F2} kg, S_ref: {db.ReferenceAreaSqM:F3} m^2, MAC: {db.MeanAerodynamicChord:F3} m)");
                EditorGUILayout.LabelField("Longitudinal Derivatives", $"CL0={db.CL0:F3}, CL_alpha={db.CLAlpha:F2}/rad, Cm0={db.Cm0:F3}, Cm_alpha={db.CmAlpha:F3}/rad, Cm_q={db.Cmq:F2}/rad");
                EditorGUILayout.LabelField("Parasitic & Induced Drag", $"CD0={db.CD0:F4}, Induced Factor k={db.InducedDragFactorK:F4} (Oswald e={db.OswaldEfficiency:F2})");
                EditorGUILayout.LabelField("Lateral-Directional", $"CY_beta={db.CYBeta:F3}, Cl_beta={db.ClBeta:F3}, Cn_beta={db.CnBeta:F3}, Cl_p={db.Clp:F2}, Cn_r={db.Cnr:F3}");
                EditorGUILayout.LabelField("Control Power", $"Cl_da={db.ClDeltaA:F3}/rad, Cm_de={db.CmDeltaE:F3}/rad, Cn_dr={db.CnDeltaR:F3}/rad");
                EditorGUILayout.LabelField("Neutral Point & Static Margin", $"x_NP = {db.NeutralPointXFrd:F3} m | Static Margin = {(db.StaticMarginFraction * 100.0):F1}% MAC");
                EditorGUILayout.LabelField("Equilibrium Trim State", $"V_trim={db.TrimAirspeedMps:F1} m/s, Alpha_trim={db.TrimAlphaDeg:F2} deg, Elev_trim={db.TrimElevatorDeg:F2} deg, Throttle={db.TrimThrottleNormalized * 100.0:F1}%");
            }
            EditorGUILayout.EndVertical();
        }

        private void DrawTabStage2MultiFidelityConfig()
        {
            EditorGUILayout.BeginVertical(EditorStyles.helpBox);
            GUILayout.Label("Stage 2: Real-Time Simulation Fidelity Level Selector", EditorStyles.boldLabel);
            EditorGUILayout.HelpBox(
                "Level 0 (Unity Rigidbody): Uses Unity PhysX FixedUpdate with Stage-1 lumped rotor Kt/Kq and bluff drag.\n" +
                "Level 1 (6-DOF Surrogate): Custom RK4 6-DOF solver driven by Stage-1 pre-baked derivatives & BEMT lookup curves.\n" +
                "Level 2 (6-DOF BEMT + Strip): Custom RK4 6-DOF solver with real-time per-rotor CCBlade BEMT & nonlinear wing strips (+/-180 deg Viterna).\n" +
                "Level 3 (Full 6-DOF + 3D VLM + Slipstream): Adds real-time 3D VLM horseshoe vortex downwash and contracting rotor slipstream tube wake coupling.",
                MessageType.None);

            if (_sceneVehicle != null)
            {
                Undo.RecordObject(_sceneVehicle, "Modify Stage 2 Fidelity Settings");
                _sceneVehicle.FidelityLevel = (SimulationFidelityLevel)EditorGUILayout.EnumPopup("Active Fidelity Level", _sceneVehicle.FidelityLevel);
                _sceneVehicle.Rk4SubStepFrequencyHz = EditorGUILayout.IntSlider("RK4 Integration Rate [Hz]", _sceneVehicle.Rk4SubStepFrequencyHz, 100, 1000);
                _sceneVehicle.ActiveConfig.EnableGroundEffect = EditorGUILayout.Toggle("Cheeseman-Bennett Ground Effect", _sceneVehicle.ActiveConfig.EnableGroundEffect);
                _sceneVehicle.ActiveConfig.EnableBladeFlappingAndRotorDrag = EditorGUILayout.Toggle("Edgewise Blade Flapping & H-Drag", _sceneVehicle.ActiveConfig.EnableBladeFlappingAndRotorDrag);
                _sceneVehicle.ActiveConfig.EnableDrydenTurbulence = EditorGUILayout.Toggle("Dryden Wind Turbulence", _sceneVehicle.ActiveConfig.EnableDrydenTurbulence);
                _sceneVehicle.ActiveConfig.TurbulenceIntensityMps = EditorGUILayout.Slider("Turbulence Intensity [m/s]", (float)_sceneVehicle.ActiveConfig.TurbulenceIntensityMps, 0f, 8f);
                EditorUtility.SetDirty(_sceneVehicle);
            }
            else
            {
                _wizardFidelity = (SimulationFidelityLevel)EditorGUILayout.EnumPopup("Default Fidelity Level", _wizardFidelity);
            }
            EditorGUILayout.EndVertical();
        }

        private void DrawTabGncMavlinkAndPx4Export()
        {
            var cfg = GetCurrentConfig();
            var db = GetCurrentBakedDb();

            EditorGUILayout.BeginVertical(EditorStyles.helpBox);
            GUILayout.Label("A. PX4 v1.14+ Control Allocation & Airframe Parameter Exporter", EditorStyles.boldLabel);
            GUILayout.Label("Replaces external Python scripts (e.g., px4_airframe.py) with native C# pseudoinverse mixer & CA_ROTOR/CA_SV_CS generation.");

            if (GUILayout.Button("Export PX4 Control Allocation Script (.params / .sh)", GUILayout.Height(28)))
            {
                string script = WrenchControlAllocator.GeneratePx4AirframeParameterScript(cfg, db);
                string path = EditorUtility.SaveFilePanel("Save PX4 Airframe Parameter Script", Application.dataPath, $"{cfg.AircraftName.Replace(' ', '_')}_px4.params", "params");
                if (!string.IsNullOrEmpty(path))
                {
                    File.WriteAllText(path, script);
                    Debug.Log($"[FlightDynamics6DoF] Exported PX4 Airframe script to: {path}");
                }
            }

            EditorGUILayout.BeginHorizontal();
            if (GUILayout.Button("Export Aircraft Spec to JSON"))
            {
                string json = JsonUtility.ToJson(cfg, true);
                string path = EditorUtility.SaveFilePanel("Save Aircraft JSON Spec", Application.dataPath, $"{cfg.AircraftName.Replace(' ', '_')}.json", "json");
                if (!string.IsNullOrEmpty(path))
                    File.WriteAllText(path, json);
            }
            if (GUILayout.Button("Import Aircraft Spec from JSON"))
            {
                string path = EditorUtility.OpenFilePanel("Import Aircraft JSON Spec", Application.dataPath, "json");
                if (!string.IsNullOrEmpty(path) && File.Exists(path))
                {
                    string json = File.ReadAllText(path);
                    var imported = JsonUtility.FromJson<AircraftConfigSpec>(json);
                    if (imported != null)
                    {
                        if (_targetProfile != null) { _targetProfile.Config = imported; _targetProfile.AnalyzeAndBake(); }
                        else if (_sceneVehicle != null) { _sceneVehicle.InlineConfig = imported; _sceneVehicle.RunStage1AnalysisAndBake(); }
                    }
                }
            }
            EditorGUILayout.EndHorizontal();
            EditorGUILayout.EndVertical();
        }

        private void DrawTabOneClickSceneSetup()
        {
            EditorGUILayout.BeginVertical(EditorStyles.helpBox);
            GUILayout.Label("One-Click Engineering Simulation Scene Builder", EditorStyles.boldLabel);
            _wizardPresetCategory = (VehicleAirframeCategory)EditorGUILayout.EnumPopup("Vehicle Airframe Preset", _wizardPresetCategory);
            _wizardFidelity = (SimulationFidelityLevel)EditorGUILayout.EnumPopup("Stage-2 Fidelity Level", _wizardFidelity);
            _wizardMavlinkMode = (MavlinkBridgeMode)EditorGUILayout.EnumPopup("MAVLink Operating Mode", _wizardMavlinkMode);
            _wizardAutoTakeoff = EditorGUILayout.Toggle("Auto-Arm & Takeoff on Play", _wizardAutoTakeoff);

            EditorGUILayout.Space(8);
            if (GUILayout.Button("Build Complete 6-DOF Flight Simulation Environment in Current Scene", GUILayout.Height(36)))
            {
                BuildCompleteSimulationScene();
            }
            EditorGUILayout.EndVertical();
        }

        private void ApplyPreset(VehicleAirframeCategory category)
        {
            if (_targetProfile != null)
            {
                Undo.RecordObject(_targetProfile, "Load Aircraft Preset");
                _targetProfile.LoadPreset(category);
                EditorUtility.SetDirty(_targetProfile);
            }
            else if (_sceneVehicle != null)
            {
                Undo.RecordObject(_sceneVehicle, "Load Aircraft Preset");
                _sceneVehicle.InlineConfig = category switch
                {
                    VehicleAirframeCategory.FixedWing => AircraftConfigSpec.CreateFixedWingUav(),
                    VehicleAirframeCategory.HybridQuadPlaneVtol => AircraftConfigSpec.CreateQuadPlaneVtolMillie(),
                    _ => AircraftConfigSpec.CreateStandardQuadcopterX()
                };
                _sceneVehicle.RunStage1AnalysisAndBake();
                EditorUtility.SetDirty(_sceneVehicle);
            }
        }

        private void RunBakeOnActiveTarget()
        {
            if (_targetProfile != null)
            {
                Undo.RecordObject(_targetProfile, "Bake Aero DB");
                _targetProfile.AnalyzeAndBake();
                EditorUtility.SetDirty(_targetProfile);
            }
            if (_sceneVehicle != null)
            {
                Undo.RecordObject(_sceneVehicle, "Bake Scene Vehicle Aero DB");
                _sceneVehicle.RunStage1AnalysisAndBake();
                EditorUtility.SetDirty(_sceneVehicle);
            }
        }

        private void CreateNewProfileAsset()
        {
            string path = EditorUtility.SaveFilePanelInProject(
                "Create Aircraft Profile Asset",
                "EngineeringAircraftProfile",
                "asset",
                "Choose location for the AircraftProfileAsset");

            if (string.IsNullOrEmpty(path)) return;

            var asset = CreateInstance<AircraftProfileAsset>();
            asset.LoadPreset(_wizardPresetCategory);
            AssetDatabase.CreateAsset(asset, path);
            AssetDatabase.SaveAssets();
            _targetProfile = asset;
            Selection.activeObject = asset;
        }

        private void BuildCompleteSimulationScene()
        {
            var mgr = FlightSimulationManager.SpawnCompleteSimulationEnvironment(
                _wizardPresetCategory,
                _wizardFidelity,
                _wizardMavlinkMode,
                _wizardAutoTakeoff);

            _sceneVehicle = mgr.PrimaryVehicle;
            if (_sceneVehicle != null)
            {
                Selection.activeGameObject = _sceneVehicle.gameObject;
                if (SceneView.lastActiveSceneView != null)
                    SceneView.lastActiveSceneView.FrameSelected();
                Debug.Log($"[FlightDynamics6DoF] Successfully created 3D simulation environment for {_sceneVehicle.ActiveConfig.AircraftName} ({_wizardFidelity}).");
            }
        }
    }
}
