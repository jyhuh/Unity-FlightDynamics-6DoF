// Licensed under the Apache License, Version 2.0.
#nullable enable
using FlightDynamics6DoF.Core.Mavlink;
using FlightDynamics6DoF.Core.Specs;
using FlightDynamics6DoF.Unity;
using UnityEditor;
using UnityEngine;

namespace FlightDynamics6DoF.Editor
{
    /// <summary>
    /// Step 3: 3D scene setup, 6-DOF fidelity selection, and wind/turbulence environment.
    /// </summary>
    internal sealed class SimulationEnvironmentPanel
    {
        public SimulationFidelityLevel Fidelity = SimulationFidelityLevel.Level2_6DoFBladeAndStrip;
        public MavlinkBridgeMode MavlinkMode = MavlinkBridgeMode.ExternalSitlHil;
        public bool AutoTakeoff;
        public bool StopContainersOnSimExit = false;

        private static readonly string[] FidelityButtonLabels = new[]
        {
            "L0: Fast Kinematics",
            "L1: 6-DOF Surrogate",
            "L2: Blade & Strip",
            "L3: Full Aero Wake"
        };

        public void Draw(FlightDynamicsEngineeringWizard host)
        {
            EditorGUILayout.BeginVertical(EditorStyles.helpBox);
            GUILayout.Label("Step 3.1: Simulation Execution & Shutdown", EditorStyles.boldLabel);
            StopContainersOnSimExit = EditorGUILayout.ToggleLeft("Stop PX4/ROS 2 containers when exiting Play Mode (default off for fast restarts)", StopContainersOnSimExit);

            EditorGUILayout.BeginHorizontal();
            GUI.enabled = !EditorApplication.isPlaying;
            GUI.backgroundColor = new Color(0.45f, 0.95f, 0.55f);
            if (GUILayout.Button("▶ Start Simulation (Unity Play + Auto-Logging)", GUILayout.Height(34)))
                host.StartSimulation();

            GUI.enabled = EditorApplication.isPlaying;
            GUI.backgroundColor = new Color(1f, 0.45f, 0.45f);
            if (GUILayout.Button("■ Stop Simulation (Stop Play + Auto-Save Logs)", GUILayout.Height(34)))
                host.StopSimulation();
            GUI.enabled = true;
            GUI.backgroundColor = Color.white;
            EditorGUILayout.EndHorizontal();
            EditorGUILayout.EndVertical();

            EditorGUILayout.Space(6);

            var cfg = host.GetCurrentConfig();

            EditorGUILayout.BeginVertical(EditorStyles.helpBox);
            GUILayout.Label("Step 3.2: 3D Scene Environment Setup (One-Click)", EditorStyles.boldLabel);
            EditorGUILayout.BeginHorizontal();
            EditorGUILayout.LabelField("Step 1 Selected Airframe", $"{cfg.Category} — {cfg.AircraftName} (PX4: {host.AirframeModel})");
            if (GUILayout.Button("Change in Step 1", GUILayout.Width(120)))
                host.SelectTab(0);
            EditorGUILayout.EndHorizontal();

            MavlinkMode = (MavlinkBridgeMode)EditorGUILayout.EnumPopup("MAVLink Bridge Mode", MavlinkMode);
            AutoTakeoff = EditorGUILayout.Toggle("Auto Takeoff on Start (Internal)", AutoTakeoff);
            if (GUILayout.Button("Build / Rebuild 6-DOF Scene Environment", GUILayout.Height(28)))
                host.BuildCompleteSimulationScene();
            EditorGUILayout.EndVertical();

            EditorGUILayout.Space(6);

            var vehicle = host.SceneVehicle;
            EditorGUILayout.BeginVertical(EditorStyles.helpBox);
            GUILayout.Label("Step 3.3: 6-DOF Physics Fidelity & Atmospheric Wind/Turbulence", EditorStyles.boldLabel);

            GUILayout.Label("Physics Fidelity Level", EditorStyles.miniBoldLabel);
            SimulationFidelityLevel activeFidelity = vehicle != null ? vehicle.FidelityLevel : Fidelity;
            SimulationFidelityLevel nextFidelity = DrawFidelityHorizontalButtons(activeFidelity);
            if (nextFidelity != activeFidelity)
            {
                Fidelity = nextFidelity;
                if (vehicle != null)
                {
                    Undo.RecordObject(vehicle, "Change Simulation Fidelity");
                    vehicle.FidelityLevel = nextFidelity;
                    EditorUtility.SetDirty(vehicle);
                }
            }

            if (vehicle != null)
            {
                Undo.RecordObject(vehicle, "Modify Stage 2 Sim Environment");
                vehicle.Rk4SubStepFrequencyHz = EditorGUILayout.IntSlider("RK4 Integration Rate [Hz]", vehicle.Rk4SubStepFrequencyHz, 100, 1000);
                vehicle.ActiveConfig.EnableGroundEffect = EditorGUILayout.Toggle("Ground Effect (IGE)", vehicle.ActiveConfig.EnableGroundEffect);
                vehicle.ActiveConfig.EnableBladeFlappingAndRotorDrag = EditorGUILayout.Toggle("Blade Flapping & H-Drag", vehicle.ActiveConfig.EnableBladeFlappingAndRotorDrag);
                vehicle.ActiveConfig.EnableDrydenTurbulence = EditorGUILayout.Toggle("Dryden Turbulence", vehicle.ActiveConfig.EnableDrydenTurbulence);
                vehicle.ActiveConfig.TurbulenceIntensityMps = EditorGUILayout.Slider("Turbulence Intensity [m/s]", (float)vehicle.ActiveConfig.TurbulenceIntensityMps, 0f, 8f);
                EditorUtility.SetDirty(vehicle);
            }

            var simMgr = Object.FindFirstObjectByType<FlightSimulationManager>();
            if (simMgr != null)
            {
                EditorGUILayout.BeginHorizontal();
                GUILayout.Label($"Mean Wind (NED): {simMgr.MeanWindNedMetersPerSec}", GUILayout.Width(210));
                if (GUILayout.Button("Calm (0 m/s)"))
                {
                    simMgr.MeanWindNedMetersPerSec = Vector3.zero;
                    if (vehicle != null) { vehicle.ActiveConfig.EnableDrydenTurbulence = false; vehicle.ActiveConfig.TurbulenceIntensityMps = 0.0; }
                }
                if (GUILayout.Button("Light (2 m/s)"))
                {
                    simMgr.MeanWindNedMetersPerSec = new Vector3(2f, 0f, 0f);
                    if (vehicle != null) { vehicle.ActiveConfig.EnableDrydenTurbulence = true; vehicle.ActiveConfig.TurbulenceIntensityMps = 0.5; }
                }
                if (GUILayout.Button("Strong (5 m/s)"))
                {
                    simMgr.MeanWindNedMetersPerSec = new Vector3(4f, 2f, 0f);
                    if (vehicle != null) { vehicle.ActiveConfig.EnableDrydenTurbulence = true; vehicle.ActiveConfig.TurbulenceIntensityMps = 1.2; }
                }
                EditorGUILayout.EndHorizontal();
            }
            EditorGUILayout.EndVertical();
        }

        private static SimulationFidelityLevel DrawFidelityHorizontalButtons(SimulationFidelityLevel current)
        {
            int curIdx = (int)current;
            EditorGUILayout.BeginHorizontal();
            for (int i = 0; i < FidelityButtonLabels.Length; i++)
            {
                bool isSelected = i == curIdx;
                GUI.backgroundColor = isSelected ? new Color(0.35f, 0.88f, 0.55f) : new Color(0.88f, 0.88f, 0.88f);
                if (GUILayout.Button(FidelityButtonLabels[i], isSelected ? EditorStyles.miniButtonMid : EditorStyles.miniButton, GUILayout.Height(28)))
                    curIdx = i;
            }
            GUI.backgroundColor = Color.white;
            EditorGUILayout.EndHorizontal();
            return (SimulationFidelityLevel)Mathf.Clamp(curIdx, 0, 3);
        }
    }
}

