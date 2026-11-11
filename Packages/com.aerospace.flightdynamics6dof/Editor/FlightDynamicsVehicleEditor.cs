// Licensed under the Apache License, Version 2.0.
using UnityEditor;
using UnityEngine;
using FlightDynamics6DoF.Core.Math;
using FlightDynamics6DoF.Unity;

namespace FlightDynamics6DoF.Editor
{
    [CustomEditor(typeof(FlightDynamicsVehicle))]
    public class FlightDynamicsVehicleEditor : UnityEditor.Editor
    {
        public override void OnInspectorGUI()
        {
            var vehicle = (FlightDynamicsVehicle)target;
            var db = vehicle.ActiveBakedDatabase;

            EditorGUILayout.BeginVertical(EditorStyles.helpBox);
            GUILayout.Label("Stage 1: Baked Aerodynamic & Stability Summary", EditorStyles.boldLabel);
            EditorGUILayout.LabelField("Aircraft Name", vehicle.ActiveConfig.AircraftName);
            EditorGUILayout.LabelField("Static Margin", $"{(db.StaticMarginFraction * 100.0):F1}% MAC (x_NP = {db.NeutralPointXFrd:F3} m)");
            EditorGUILayout.LabelField("Derivatives", $"CL_alpha={db.CLAlpha:F2}, Cm_alpha={db.CmAlpha:F3}, Cn_beta={db.CnBeta:F3}");
            EditorGUILayout.LabelField("Drag & Trim", $"CD0={db.CD0:F4}, Trim V={db.TrimAirspeedMps:F1} m/s, Trim Alpha={db.TrimAlphaDeg:F2} deg");

            EditorGUILayout.BeginHorizontal();
            if (GUILayout.Button("Re-Analyze & Bake (Stage 1)"))
            {
                Undo.RecordObject(vehicle, "Bake Aero DB");
                vehicle.RunStage1AnalysisAndBake();
                EditorUtility.SetDirty(vehicle);
            }
            if (GUILayout.Button("Open Engineering Workbench"))
            {
                FlightDynamicsEngineeringWizard.ShowWindow();
            }
            EditorGUILayout.EndHorizontal();
            EditorGUILayout.EndVertical();

            if (Application.isPlaying && vehicle.Engine != null && vehicle.FlightController != null)
            {
                EditorGUILayout.BeginVertical(EditorStyles.helpBox);
                GUILayout.Label("Stage 2 Runtime Flight Control Commands", EditorStyles.boldLabel);
                EditorGUILayout.BeginHorizontal();
                if (GUILayout.Button("Takeoff 10m"))
                {
                    Vec3d euler = vehicle.Engine.State.AttitudeBodyToNed.ToEulerZyx();
                    vehicle.FlightController.CommandTakeoff(vehicle.Engine.State.PositionNed, euler.Z, 10.0);
                }
                if (GUILayout.Button("RTL"))
                    vehicle.FlightController.CommandReturnToLaunch();
                if (GUILayout.Button("Land"))
                    vehicle.FlightController.CommandAutoLand(vehicle.Engine.State.PositionNed);
                if (GUILayout.Button("Disarm"))
                    vehicle.FlightController.Disarm();
                EditorGUILayout.EndHorizontal();
                EditorGUILayout.EndVertical();
            }

            EditorGUILayout.Space(4);
            DrawDefaultInspector();
        }
    }
}
