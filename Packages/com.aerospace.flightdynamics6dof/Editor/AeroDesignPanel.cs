// Licensed under the Apache License, Version 2.0.
#nullable enable
using System.Diagnostics;
using System.IO;
using FlightDynamics6DoF.Core.Aerodynamics;
using FlightDynamics6DoF.Core.Math;
using FlightDynamics6DoF.Core.Specs;
using FlightDynamics6DoF.Unity;
using UnityEditor;
using UnityEngine;
using Debug = UnityEngine.Debug;

namespace FlightDynamics6DoF.Editor
{
    /// <summary>
    /// Step 1: Airframe selection, Stage-1 aerodynamic baking, rotor BEMT, and 2D airfoil calculators.
    /// </summary>
    internal sealed class AeroDesignPanel
    {
        private string _nacaCode = "2412";
        private float _testAlphaDeg = 5.0f;
        private float _testReynolds = 500000f;
        private float _testMach = 0.08f;
        private AirfoilPolarPoint _lastAirfoilPoint;
        private bool _hasAirfoilResult;

        private float _testRotorRpm = 6500f;
        private float _testRotorAxialSpeedMps = 0.0f;
        private float _testRotorAglMeters = 10.0f;
        private RotorBemtSolution _lastRotorSolution;
        private bool _hasRotorResult;

        private string _lastStage1Summary = "";

        public void Draw(FlightDynamicsEngineeringWizard host)
        {
            var cfg = host.GetCurrentConfig();
            var db = host.GetCurrentBakedDb();

            EditorGUILayout.BeginVertical(EditorStyles.helpBox);
            GUILayout.Label("Step 1.1: Select Aircraft Type & Load Airframe Spec", EditorStyles.boldLabel);
            EditorGUILayout.BeginHorizontal();
            GUI.backgroundColor = cfg.Category == VehicleAirframeCategory.Multirotor ? new Color(0.65f, 1f, 0.7f) : Color.white;
            if (GUILayout.Button("Multirotor (Quad X-450 / iris)", GUILayout.Height(28)))
                host.SwitchAirframeEverywhere(VehicleAirframeCategory.Multirotor, "iris");

            GUI.backgroundColor = cfg.Category == VehicleAirframeCategory.FixedWing ? new Color(0.65f, 1f, 0.7f) : Color.white;
            if (GUILayout.Button("Fixed-Wing (Talon-180 / plane)", GUILayout.Height(28)))
                host.SwitchAirframeEverywhere(VehicleAirframeCategory.FixedWing, "plane");

            GUI.backgroundColor = (cfg.Category == VehicleAirframeCategory.HybridQuadPlaneVtol || cfg.Category == VehicleAirframeCategory.TiltRotorVtol)
                ? new Color(0.65f, 1f, 0.7f) : Color.white;
            if (GUILayout.Button("VTOL (QuadPlane / standard_vtol)", GUILayout.Height(28)))
                host.SwitchAirframeEverywhere(VehicleAirframeCategory.HybridQuadPlaneVtol, "standard_vtol");
            GUI.backgroundColor = Color.white;
            EditorGUILayout.EndHorizontal();

            EditorGUILayout.Space(4);
            EditorGUILayout.BeginHorizontal();
            if (GUILayout.Button("Run Stage-1 Aero Analysis & Bake DB", GUILayout.Height(28)))
            {
                RunBakeOnActiveTarget(host);
                db = host.GetCurrentBakedDb();
            }
            if (host.SceneVehicle != null && GUILayout.Button("Extract Mass/Inertia from 3D Meshes", GUILayout.Height(28)))
            {
                Undo.RecordObject(host.SceneVehicle, "Extract 3D Mesh Geometry");
                host.SceneVehicle.ExtractGeometryFromChildMeshesAndBake();
                EditorUtility.SetDirty(host.SceneVehicle);
                db = host.GetCurrentBakedDb();
            }
            if (GUILayout.Button("Save Spec JSON", GUILayout.Width(115), GUILayout.Height(28)))
            {
                string path = EditorUtility.SaveFilePanel("Save Aircraft JSON", Application.dataPath, $"{cfg.AircraftName.Replace(' ', '_')}.json", "json");
                if (!string.IsNullOrEmpty(path)) File.WriteAllText(path, JsonUtility.ToJson(cfg, true));
            }
            if (GUILayout.Button("Import JSON", GUILayout.Width(105), GUILayout.Height(28)))
            {
                string path = EditorUtility.OpenFilePanel("Import Aircraft JSON", Application.dataPath, "json");
                if (!string.IsNullOrEmpty(path) && File.Exists(path))
                {
                    var imported = JsonUtility.FromJson<AircraftConfigSpec>(File.ReadAllText(path));
                    if (imported != null && host.SceneVehicle != null)
                    {
                        host.SceneVehicle.InlineConfig = imported;
                        host.SceneVehicle.RunStage1AnalysisAndBake();
                        host.SyncEditingParamTableFromVehicle();
                    }
                }
            }
            EditorGUILayout.EndHorizontal();
            if (!string.IsNullOrEmpty(_lastStage1Summary))
                EditorGUILayout.HelpBox(_lastStage1Summary, MessageType.Info);
            EditorGUILayout.EndVertical();

            EditorGUILayout.Space(6);

            if (cfg.Category == VehicleAirframeCategory.Multirotor)
            {
                DrawMultirotorDesignPanel(host, cfg, db);
            }
            else if (cfg.Category == VehicleAirframeCategory.FixedWing)
            {
                DrawFixedWingDesignPanel(host, cfg, db);
            }
            else
            {
                DrawMultirotorDesignPanel(host, cfg, db);
                EditorGUILayout.Space(6);
                DrawFixedWingDesignPanel(host, cfg, db);
            }
        }

        public static void CreateNewProfileAsset(FlightDynamicsEngineeringWizard host)
        {
            string path = EditorUtility.SaveFilePanelInProject("Create Aircraft Profile Asset", "EngineeringAircraftProfile", "asset", "Choose location");
            if (string.IsNullOrEmpty(path)) return;
            var asset = ScriptableObject.CreateInstance<AircraftProfileAsset>();
            asset.LoadPreset(host.PresetCategory);
            AssetDatabase.CreateAsset(asset, path);
            AssetDatabase.SaveAssets();
            host.TargetProfile = asset;
            Selection.activeObject = asset;
        }

        private void DrawMultirotorDesignPanel(FlightDynamicsEngineeringWizard host, AircraftConfigSpec cfg, BakedAeroDatabase db)
        {
            EditorGUILayout.BeginVertical(EditorStyles.helpBox);
            GUILayout.Label($"[Multirotor Propulsion & Aero Design] {cfg.AircraftName} (Rotors: {cfg.Rotors.Count})", EditorStyles.boldLabel);

            EditorGUI.BeginChangeCheck();
            cfg.MassKg = System.Math.Max(0.2, EditorGUILayout.FloatField("Total Mass [kg]", (float)cfg.MassKg));
            if (cfg.Rotors.Count > 0)
            {
                var r0 = cfg.Rotors[0];
                float radiusM = EditorGUILayout.Slider("Rotor Radius R [m]", (float)r0.Radius, 0.05f, 0.80f);
                int blades = EditorGUILayout.IntSlider("Blade Count", r0.NumBlades, 2, 6);
                float maxRpm = EditorGUILayout.Slider("Max RPM", (float)r0.MaxRpm, 3000f, 25000f);
                float idleRpm = EditorGUILayout.Slider("Idle RPM", (float)r0.IdleRpm, 300f, maxRpm * 0.5f);
                if (EditorGUI.EndChangeCheck())
                {
                    foreach (var r in cfg.Rotors)
                    {
                        r.Radius = radiusM;
                        r.NumBlades = blades;
                        r.MaxRpm = maxRpm;
                        r.IdleRpm = idleRpm;
                    }
                    host.RebakeActiveConfig(cfg);
                    db = host.GetCurrentBakedDb();
                }
            }
            else
            {
                EditorGUI.EndChangeCheck();
            }

            var atmDesign = IsaAtmosphereAndTurbulence.Evaluate(cfg.DesignAltitudeMsl);
            double totalMaxThrustN = 0.0;
            foreach (var r in cfg.Rotors)
                totalMaxThrustN += CcBladeBemtSolver.EvaluateRotor(r, Vec3d.Zero, Vec3d.Zero, r.MaxRpm, atmDesign, 10.0, false, false).ThrustNewtons;
            double weightN = cfg.MassKg * 9.80665;
            double twRatio = weightN > 1e-6 ? totalMaxThrustN / weightN : 0.0;

            EditorGUILayout.Space(4);
            EditorGUILayout.LabelField("Inertia Tensor [kg·m²]", $"Ixx={cfg.Ixx:F4}, Iyy={cfg.Iyy:F4}, Izz={cfg.Izz:F4}");
            EditorGUILayout.LabelField("Propulsion Summary", $"Weight={weightN:F1} N | Max Total Thrust={totalMaxThrustN:F1} N (T/W={twRatio:F2}) | Hover Throttle={db.TrimThrottleNormalized * 100.0:F1}%");

            EditorGUILayout.Space(4);
            GUILayout.Label("Module 4: CCBlade BEMT Rotor Thrust / Efficiency / Ground Effect (IGE) Solver", EditorStyles.miniBoldLabel);
            _testRotorRpm = EditorGUILayout.Slider("Rotor Speed [RPM]", _testRotorRpm, 500f, 15000f);
            _testRotorAxialSpeedMps = EditorGUILayout.Slider("Axial Inflow V_inf [m/s]", _testRotorAxialSpeedMps, -5f, 35f);
            _testRotorAglMeters = EditorGUILayout.Slider("Altitude AGL [m]", _testRotorAglMeters, 0.1f, 20f);
            if (GUILayout.Button("Compute Rotor BEMT Performance (incl. Ground Effect)"))
            {
                RotorSpec rSpec = cfg.Rotors.Count > 0 ? cfg.Rotors[0] : AircraftConfigSpec.CreateStandardQuadcopterX().Rotors[0];
                _lastRotorSolution = CcBladeBemtSolver.EvaluateRotor(rSpec, rSpec.ThrustAxisFrd.Normalized() * _testRotorAxialSpeedMps, Vec3d.Zero, _testRotorRpm, atmDesign, _testRotorAglMeters, true, true);
                _hasRotorResult = true;
            }
            if (_hasRotorResult)
            {
                EditorGUILayout.HelpBox(
                    $"Single Rotor Thrust={_lastRotorSolution.ThrustNewtons:F2} N (Total {cfg.Rotors.Count}x={_lastRotorSolution.ThrustNewtons * cfg.Rotors.Count:F1} N) | Torque={_lastRotorSolution.AerodynamicTorqueNm:F4} Nm | Shaft Power={_lastRotorSolution.ShaftPowerWatts:F1} W | Eta={(_lastRotorSolution.EfficiencyEta * 100.0):F1}%",
                    MessageType.Info);
            }
            EditorGUILayout.EndVertical();
        }

        private void DrawFixedWingDesignPanel(FlightDynamicsEngineeringWizard host, AircraftConfigSpec cfg, BakedAeroDatabase db)
        {
            EditorGUILayout.BeginVertical(EditorStyles.helpBox);
            GUILayout.Label($"[Fixed-Wing Aero & Stability Design] {cfg.AircraftName}", EditorStyles.boldLabel);

            EditorGUI.BeginChangeCheck();
            cfg.MassKg = System.Math.Max(0.2, EditorGUILayout.FloatField("Total Mass [kg]", (float)cfg.MassKg));
            cfg.ReferenceAreaSqM = System.Math.Max(0.05, EditorGUILayout.FloatField("Reference Wing Area S_ref [m²]", (float)cfg.ReferenceAreaSqM));
            cfg.ReferenceChordMeters = System.Math.Max(0.05, EditorGUILayout.FloatField("Mean Aero Chord MAC [m]", (float)cfg.ReferenceChordMeters));
            cfg.ReferenceSpanMeters = System.Math.Max(0.2, EditorGUILayout.FloatField("Wing Span b [m]", (float)cfg.ReferenceSpanMeters));
            cfg.DesignCruiseSpeedMps = System.Math.Max(5.0, EditorGUILayout.FloatField("Design Cruise Speed [m/s]", (float)cfg.DesignCruiseSpeedMps));
            if (EditorGUI.EndChangeCheck())
            {
                host.RebakeActiveConfig(cfg);
                db = host.GetCurrentBakedDb();
            }

            if (db != null && !string.IsNullOrEmpty(db.AircraftName))
            {
                EditorGUILayout.Space(4);
                EditorGUILayout.LabelField("Longitudinal Derivatives", $"CL0={db.CL0:F3}, CL_alpha={db.CLAlpha:F2}/rad, Cm0={db.Cm0:F3}, Cm_alpha={db.CmAlpha:F3}/rad, Cm_q={db.Cmq:F2}");
                EditorGUILayout.LabelField("Drag Polar", $"CD0={db.CD0:F4}, k={db.InducedDragFactorK:F4} (Oswald e={db.OswaldEfficiency:F2})");
                EditorGUILayout.LabelField("Lateral-Directional Derivatives", $"CY_beta={db.CYBeta:F3}, Cl_beta={db.ClBeta:F3}, Cn_beta={db.CnBeta:F3}, Cl_p={db.Clp:F2}, Cn_r={db.Cnr:F3}");
                EditorGUILayout.LabelField("Control Power", $"Cl_da={db.ClDeltaA:F3}/rad, Cm_de={db.CmDeltaE:F3}/rad, Cn_dr={db.CnDeltaR:F3}/rad");
                EditorGUILayout.LabelField("Neutral Point & Static Margin", $"x_NP={db.NeutralPointXFrd:F3} m | Static Margin={(db.StaticMarginFraction * 100.0):F1}% MAC | Stable={(db.IsLongitudinallyStable && db.IsDirectionallyStable)}");
                EditorGUILayout.LabelField("Trim Condition", $"V_trim={db.TrimAirspeedMps:F1} m/s | Alpha_trim={db.TrimAlphaDeg:F2} deg | Elev_trim={db.TrimElevatorDeg:F2} deg");
            }

            EditorGUILayout.Space(4);
            GUILayout.Label("Module 1: 2D Airfoil Viscous Polar & Viterna (±180°) Solver", EditorStyles.miniBoldLabel);
            _nacaCode = EditorGUILayout.TextField("NACA 4-Digit Code", _nacaCode);
            _testAlphaDeg = EditorGUILayout.Slider("Angle of Attack Alpha [deg]", _testAlphaDeg, -180f, 180f);
            _testReynolds = EditorGUILayout.Slider("Reynolds Number (Re)", _testReynolds, 1.0e4f, 1.0e7f);
            _testMach = EditorGUILayout.Slider("Mach Number (M < 0.7)", _testMach, 0.0f, 0.68f);
            if (GUILayout.Button("Solve 2D Airfoil Polar"))
            {
                var af = AirfoilSpec.FromNaca4Digit(_nacaCode, _testReynolds);
                _lastAirfoilPoint = AirfoilPolarSolver.Evaluate(af, _testAlphaDeg * FrameConverter.Deg2Rad, _testReynolds, _testMach);
                _hasAirfoilResult = true;
            }
            if (_hasAirfoilResult)
            {
                EditorGUILayout.HelpBox(
                    $"Alpha={_lastAirfoilPoint.AlphaDeg:F1} deg | Cl={_lastAirfoilPoint.Cl:F4} | Cd={_lastAirfoilPoint.Cd:F4} | Cm={_lastAirfoilPoint.Cm:F4} | L/D={(_lastAirfoilPoint.Cl / System.Math.Max(1e-4, _lastAirfoilPoint.Cd)):F1}",
                    MessageType.Info);
            }
            EditorGUILayout.EndVertical();
        }

        private void RunBakeOnActiveTarget(FlightDynamicsEngineeringWizard host)
        {
            var sw = Stopwatch.StartNew();
            var cfg = host.GetCurrentConfig();
            string targetDesc = host.RebakeActiveConfig(cfg);
            sw.Stop();

            var db = host.GetCurrentBakedDb();
            string summary =
                $"[Stage-1 Aero Analysis & DB Bake Complete] Target={targetDesc} | Aircraft={cfg.AircraftName} ({cfg.Category}) | " +
                $"Elapsed={sw.Elapsed.TotalMilliseconds:F1} ms | " +
                $"TrimThrottle={db.TrimThrottleNormalized * 100.0:F1}%, CL_alpha={db.CLAlpha:F2}/rad, Cm_alpha={db.CmAlpha:F3}/rad, " +
                $"StaticMargin={db.StaticMarginFraction * 100.0:F1}% MAC, V_trim={db.TrimAirspeedMps:F1} m/s";

            _lastStage1Summary = summary;
            Debug.Log($"[FlightDynamics6DoF] {summary}");
        }
    }
}

