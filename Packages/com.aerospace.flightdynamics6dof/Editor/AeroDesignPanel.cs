// Licensed under the Apache License, Version 2.0.
#nullable enable
using System.IO;
using FlightDynamics6DoF.Core.Aerodynamics;
using FlightDynamics6DoF.Core.Math;
using FlightDynamics6DoF.Core.Specs;
using FlightDynamics6DoF.Unity;
using UnityEditor;
using UnityEngine;

namespace FlightDynamics6DoF.Editor
{
    /// <summary>
    /// Step 1: Airframe selection, JSON Save/Import, Designer popup launcher, and read-only aircraft spec overview.
    /// Detailed geometry editing, BEMT/Airfoil solvers, and Stage-1 aero baking live in <see cref="AirframeDesignerWindow"/>.
    /// </summary>
    internal sealed class AeroDesignPanel
    {
        public void Draw(FlightDynamicsEngineeringWizard host)
        {
            var cfg = host.GetCurrentConfig();
            var db = host.GetCurrentBakedDb();

            EditorGUILayout.BeginVertical(EditorStyles.helpBox);
            GUILayout.Label("Step 1.1: Select Aircraft Type & Manage Airframe Spec", EditorStyles.boldLabel);
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
            GUI.backgroundColor = new Color(0.55f, 0.85f, 1.0f);
            if (GUILayout.Button("Open Drone Shape, Dimension & Spec Designer...", GUILayout.Height(30)))
            {
                AirframeDesignerWindow.Open(host);
            }
            GUI.backgroundColor = Color.white;

            if (GUILayout.Button("Save Spec JSON", GUILayout.Width(115), GUILayout.Height(30)))
            {
                string path = EditorUtility.SaveFilePanel("Save Aircraft JSON", Application.dataPath, $"{cfg.AircraftName.Replace(' ', '_')}.json", "json");
                if (!string.IsNullOrEmpty(path)) File.WriteAllText(path, JsonUtility.ToJson(cfg, true));
            }
            if (GUILayout.Button("Import JSON", GUILayout.Width(105), GUILayout.Height(30)))
            {
                string path = EditorUtility.OpenFilePanel("Import Aircraft JSON", Application.dataPath, "json");
                if (!string.IsNullOrEmpty(path) && File.Exists(path))
                {
                    var imported = JsonUtility.FromJson<AircraftConfigSpec>(File.ReadAllText(path));
                    if (imported != null)
                    {
                        if (host.GetCurrentConfig().Category != imported.Category)
                        {
                            string model = imported.Category switch
                            {
                                VehicleAirframeCategory.FixedWing => "plane",
                                VehicleAirframeCategory.HybridQuadPlaneVtol => "standard_vtol",
                                _ => "iris"
                            };
                            host.SwitchAirframeEverywhere(imported.Category, model);
                        }
                        host.RebakeActiveConfig(imported);
                        host.SyncEditingParamTableFromVehicle();
                        cfg = host.GetCurrentConfig();
                        db = host.GetCurrentBakedDb();
                    }
                }
            }
            EditorGUILayout.EndHorizontal();
            EditorGUILayout.EndVertical();

            EditorGUILayout.Space(6);
            EditorGUILayout.BeginHorizontal();
            EditorGUILayout.BeginVertical(GUILayout.Width(340f));
            AirframeDesignerWindow.DrawBlueprintPreview(cfg, 320f, 360f, expandHeight: false);
            EditorGUILayout.EndVertical();

            EditorGUILayout.BeginVertical(GUILayout.ExpandWidth(true));
            DrawCurrentSpecSummary(cfg, db);
            EditorGUILayout.EndVertical();
            EditorGUILayout.EndHorizontal();
        }

        private static void DrawCurrentSpecSummary(AircraftConfigSpec cfg, BakedAeroDatabase db)
        {
            // 1. Identity & Dimensions
            EditorGUILayout.BeginVertical(EditorStyles.helpBox);
            GUILayout.Label($"Current Airframe Spec — {cfg.AircraftName}", EditorStyles.boldLabel);
            EditorGUILayout.LabelField("Category", cfg.Category.ToString());
            EditorGUILayout.LabelField("Fuselage", $"Length {cfg.Fuselage.Length:F2} m × Diameter {cfg.Fuselage.MaxDiameter:F2} m | Gear Height {cfg.LandingGearHeightMeters:F2} m");

            if (cfg.Category == VehicleAirframeCategory.Multirotor)
            {
                EditorGUILayout.LabelField("Wheelbase / Span", $"{cfg.ReferenceSpanMeters:F2} m");
            }
            else
            {
                EditorGUILayout.LabelField("Wing Geometry", $"Span b={cfg.ReferenceSpanMeters:F2} m | MAC={cfg.ReferenceChordMeters:F2} m | S_ref={cfg.ReferenceAreaSqM:F3} m² | Cruise={cfg.DesignCruiseSpeedMps:F1} m/s");
            }

            if (!string.IsNullOrEmpty(cfg.SpecNotes))
                EditorGUILayout.HelpBox($"Spec Notes:\n{cfg.SpecNotes}", MessageType.None);
            EditorGUILayout.EndVertical();

            EditorGUILayout.Space(4);

            // 2. Weight, Propulsion & Battery
            EditorGUILayout.BeginVertical(EditorStyles.helpBox);
            GUILayout.Label("Weight, Propulsion & Battery", EditorStyles.boldLabel);
            double weightN = cfg.MassKg * 9.80665;
            EditorGUILayout.LabelField("Total Mass & CG", $"{cfg.MassKg:F2} kg ({weightN:F1} N) | CG_x={cfg.CenterOfGravityFrd.X:F3} m");
            EditorGUILayout.LabelField("Inertia Tensor [kg·m²]", $"Ixx={cfg.Ixx:F4}, Iyy={cfg.Iyy:F4}, Izz={cfg.Izz:F4}");

            if (cfg.Rotors.Count > 0)
            {
                var r0 = cfg.Rotors[0];
                var atmDesign = IsaAtmosphereAndTurbulence.Evaluate(cfg.DesignAltitudeMsl);
                double totalMaxThrustN = 0.0;
                foreach (var r in cfg.Rotors)
                    totalMaxThrustN += CcBladeBemtSolver.EvaluateRotor(r, Vec3d.Zero, Vec3d.Zero, r.MaxRpm, atmDesign, 10.0, false, false).ThrustNewtons;
                double twRatio = weightN > 1e-6 ? totalMaxThrustN / weightN : 0.0;

                EditorGUILayout.LabelField("Motors & Rotors",
                    $"{cfg.Rotors.Count}× {r0.MaxMotorPowerWatts:F0}W ({r0.MotorKv:F0} KV) | R={r0.Radius:F3} m ({r0.NumBlades}-blade) | Max {r0.MaxRpm:F0} RPM");
                EditorGUILayout.LabelField("Thrust Capability",
                    $"Max Total Thrust={totalMaxThrustN:F1} N (T/W={twRatio:F2}) | Hover Throttle={db.TrimThrottleNormalized * 100.0:F1}%");
            }

            double nominalV = cfg.BatteryCells * 3.7;
            double energyWh = nominalV * (cfg.BatteryCapacityMah / 1000.0);
            EditorGUILayout.LabelField("Battery Pack",
                $"{cfg.BatteryCells:F0}S ({nominalV:F1} V) | {cfg.BatteryCapacityMah:F0} mAh ({energyWh:F1} Wh) | R_int={cfg.BatteryInternalResistanceOhm:F3} Ω");
            EditorGUILayout.EndVertical();

            // 3. Baked Stage-1 Aerodynamic Summary
            if (db != null && !string.IsNullOrEmpty(db.AircraftName))
            {
                EditorGUILayout.Space(4);
                EditorGUILayout.BeginVertical(EditorStyles.helpBox);
                GUILayout.Label("Baked Stage-1 Aerodynamic & Trim Summary", EditorStyles.boldLabel);
                EditorGUILayout.LabelField("Longitudinal", $"CL0={db.CL0:F3}, CL_alpha={db.CLAlpha:F2}/rad, Cm0={db.Cm0:F3}, Cm_alpha={db.CmAlpha:F3}/rad, Cm_q={db.Cmq:F2}");
                EditorGUILayout.LabelField("Drag Polar", $"CD0={db.CD0:F4}, k={db.InducedDragFactorK:F4} (Oswald e={db.OswaldEfficiency:F2})");
                EditorGUILayout.LabelField("Stability Margin", $"x_NP={db.NeutralPointXFrd:F3} m | Static Margin={(db.StaticMarginFraction * 100.0):F1}% MAC | Stable={(db.IsLongitudinallyStable && db.IsDirectionallyStable)}");
                EditorGUILayout.LabelField("Trim Condition", $"V_trim={db.TrimAirspeedMps:F1} m/s | Alpha_trim={db.TrimAlphaDeg:F2} deg | Elev_trim={db.TrimElevatorDeg:F2} deg");
                EditorGUILayout.EndVertical();
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
    }
}
