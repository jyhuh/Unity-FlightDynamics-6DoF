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
            GUILayout.Label("Step 3.1: 시뮬레이션 실행 및 종료", EditorStyles.boldLabel);
            StopContainersOnSimExit = EditorGUILayout.ToggleLeft("시뮬레이션 종료 시 PX4/ROS2 컨테이너도 함께 중지 (기본 해제: 빠른 재시작을 위해 컨테이너 유지)", StopContainersOnSimExit);

            EditorGUILayout.BeginHorizontal();
            GUI.enabled = !EditorApplication.isPlaying;
            GUI.backgroundColor = new Color(0.45f, 0.95f, 0.55f);
            if (GUILayout.Button("▶ 시뮬레이션 시작 (Unity Play + 자동 로깅 시작)", GUILayout.Height(34)))
                host.StartSimulation();

            GUI.enabled = EditorApplication.isPlaying;
            GUI.backgroundColor = new Color(1f, 0.45f, 0.45f);
            if (GUILayout.Button("■ 시뮬레이션 종료 (Play 중지 + Assets/FlightLogs 자동 저장)", GUILayout.Height(34)))
                host.StopSimulation();
            GUI.enabled = true;
            GUI.backgroundColor = Color.white;
            EditorGUILayout.EndHorizontal();
            EditorGUILayout.EndVertical();

            EditorGUILayout.Space(6);

            var cfg = host.GetCurrentConfig();

            EditorGUILayout.BeginVertical(EditorStyles.helpBox);
            GUILayout.Label("Step 3.2: 3D 씬 환경 구성 (One-Click Scene Setup)", EditorStyles.boldLabel);
            EditorGUILayout.BeginHorizontal();
            EditorGUILayout.LabelField("Step 1 선택 기체 (자동 연동)", $"{cfg.Category} — {cfg.AircraftName} (PX4: {host.AirframeModel})");
            if (GUILayout.Button("Step 1에서 변경", GUILayout.Width(115)))
                host.SelectTab(0);
            EditorGUILayout.EndHorizontal();

            MavlinkMode = (MavlinkBridgeMode)EditorGUILayout.EnumPopup("MAVLink 연결 모드", MavlinkMode);
            AutoTakeoff = EditorGUILayout.Toggle("시작 시 자동 이륙 (내부 제어기)", AutoTakeoff);
            if (GUILayout.Button("현재 씬에 6-DOF 시뮬레이션 환경 생성 / 재구성", GUILayout.Height(28)))
                host.BuildCompleteSimulationScene();
            EditorGUILayout.EndVertical();

            EditorGUILayout.Space(6);

            var vehicle = host.SceneVehicle;
            EditorGUILayout.BeginVertical(EditorStyles.helpBox);
            GUILayout.Label("Step 3.3: 6-DOF 물리 충실도 & 대기 바람·난류 환경", EditorStyles.boldLabel);

            GUILayout.Label("물리 충실도 (Fidelity Level — 단일 선택)", EditorStyles.miniBoldLabel);
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
                vehicle.Rk4SubStepFrequencyHz = EditorGUILayout.IntSlider("RK4 적분 주파수 [Hz]", vehicle.Rk4SubStepFrequencyHz, 100, 1000);
                vehicle.ActiveConfig.EnableGroundEffect = EditorGUILayout.Toggle("지면 효과 (Ground Effect)", vehicle.ActiveConfig.EnableGroundEffect);
                vehicle.ActiveConfig.EnableBladeFlappingAndRotorDrag = EditorGUILayout.Toggle("블레이드 플래핑 & H-Drag", vehicle.ActiveConfig.EnableBladeFlappingAndRotorDrag);
                vehicle.ActiveConfig.EnableDrydenTurbulence = EditorGUILayout.Toggle("Dryden 대기 난류", vehicle.ActiveConfig.EnableDrydenTurbulence);
                vehicle.ActiveConfig.TurbulenceIntensityMps = EditorGUILayout.Slider("난류 강도 [m/s]", (float)vehicle.ActiveConfig.TurbulenceIntensityMps, 0f, 8f);
                EditorUtility.SetDirty(vehicle);
            }

            var simMgr = Object.FindFirstObjectByType<FlightSimulationManager>();
            if (simMgr != null)
            {
                EditorGUILayout.BeginHorizontal();
                GUILayout.Label($"평균 바람 (NED): {simMgr.MeanWindNedMetersPerSec}", GUILayout.Width(210));
                if (GUILayout.Button("무풍 (0 m/s)"))
                {
                    simMgr.MeanWindNedMetersPerSec = Vector3.zero;
                    if (vehicle != null) { vehicle.ActiveConfig.EnableDrydenTurbulence = false; vehicle.ActiveConfig.TurbulenceIntensityMps = 0.0; }
                }
                if (GUILayout.Button("미풍 (2 m/s)"))
                {
                    simMgr.MeanWindNedMetersPerSec = new Vector3(2f, 0f, 0f);
                    if (vehicle != null) { vehicle.ActiveConfig.EnableDrydenTurbulence = true; vehicle.ActiveConfig.TurbulenceIntensityMps = 0.5; }
                }
                if (GUILayout.Button("강풍 (5 m/s)"))
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

