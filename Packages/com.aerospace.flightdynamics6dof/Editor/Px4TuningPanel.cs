// Licensed under the Apache License, Version 2.0.
#nullable enable
using System.IO;
using FlightDynamics6DoF.Core.Aerodynamics;
using FlightDynamics6DoF.Core.Gnc;
using FlightDynamics6DoF.Core.Specs;
using FlightDynamics6DoF.Unity;
using UnityEditor;
using UnityEngine;

namespace FlightDynamics6DoF.Editor
{
    /// <summary>
    /// Step 4: Per-airframe PX4 &amp; GNC parameter tuning table and pxh/offboard commands.
    /// </summary>
    internal sealed class Px4TuningPanel
    {
        public AirframeControlParameterTable? EditingParamTable;

        private Vector2 _paramScrollPos;
        private string _customPxhCommand = "commander takeoff";
        private float _offboardAltitude = 10.0f;
        private float _offboardRadius = 8.0f;

        public string SyncTableForCategory(VehicleAirframeCategory category, AircraftConfigSpec cfg, BakedAeroDatabase db, AirframeControlParameterTable? preferred = null)
        {
            if (preferred != null && preferred.Category == category)
            {
                EditingParamTable = preferred;
            }
            else if (EditingParamTable == null || EditingParamTable.Category != category)
            {
                EditingParamTable = AirframeControlParameterTable.CreateForCategory(category, cfg, db);
            }
            return EditingParamTable.Px4SitlModelName;
        }

        public void ResetTableForCategory(VehicleAirframeCategory category, AircraftConfigSpec cfg, BakedAeroDatabase db, FlightDynamicsVehicle? vehicle)
        {
            EditingParamTable = AirframeControlParameterTable.CreateForCategory(category, cfg, db);
            ApplyToVehicle(vehicle);
        }

        public void Draw(FlightDynamicsEngineeringWizard host)
        {
            var cfg = host.GetCurrentConfig();
            var db = host.GetCurrentBakedDb();

            if (EditingParamTable == null || EditingParamTable.Category != cfg.Category)
            {
                EditingParamTable = AirframeControlParameterTable.CreateForCategory(cfg.Category, cfg, db);
                host.AirframeModel = EditingParamTable.Px4SitlModelName;
                ApplyToVehicle(host.SceneVehicle);
            }

            EditorGUILayout.BeginVertical(EditorStyles.helpBox);
            EditorGUILayout.BeginHorizontal();
            GUILayout.Label($"Step 4.1: 기체별 PX4 & GNC 파라미터 테이블 [{EditingParamTable.Category} / PX4: {EditingParamTable.Px4SitlModelName}]", EditorStyles.boldLabel);
            GUILayout.FlexibleSpace();
            if (GUILayout.Button("기체 기본값 복원", GUILayout.Width(135)))
                ResetTableForCategory(cfg.Category, cfg, db, host.SceneVehicle);
            EditorGUILayout.EndHorizontal();

            string tuningHint = cfg.Category switch
            {
                VehicleAirframeCategory.FixedWing => "고정익 모드: FW_R_TC / FW_P_TC / TECS_SPDWEIGHT / 활주로 이륙 피치 및 순항 트림 파라미터가 활성화됩니다.",
                VehicleAirframeCategory.HybridQuadPlaneVtol or VehicleAirframeCategory.TiltRotorVtol => "VTOL 모드: 멀티로터(MC_*) 호버 게인 + 고정익(FW_*) 순항 게인 + 천이 속도 파라미터가 함께 활성화됩니다.",
                _ => "멀티로터 모드: MC_ROLL_P / MC_YAW_P / MPC_XY_P / MPC_Z_P / 호버 트림 스로틀 파라미터가 활성화됩니다."
            };
            EditorGUILayout.HelpBox(tuningHint, MessageType.Info);

            EditorGUILayout.BeginHorizontal(EditorStyles.toolbar);
            GUILayout.Label("Parameter", EditorStyles.boldLabel, GUILayout.Width(135));
            GUILayout.Label("Value", EditorStyles.boldLabel, GUILayout.Width(80));
            GUILayout.Label("Tuning Slider", EditorStyles.boldLabel, GUILayout.ExpandWidth(true));
            GUILayout.Label("Unit", EditorStyles.boldLabel, GUILayout.Width(45));
            GUILayout.Label("Description", EditorStyles.boldLabel, GUILayout.Width(210));
            EditorGUILayout.EndHorizontal();

            _paramScrollPos = EditorGUILayout.BeginScrollView(_paramScrollPos, GUILayout.Height(240));
            for (int i = 0; i < EditingParamTable.Entries.Count; i++)
            {
                var entry = EditingParamTable.Entries[i];
                EditorGUILayout.BeginHorizontal();
                GUILayout.Label(entry.Name, EditorStyles.miniBoldLabel, GUILayout.Width(135));
                EditorGUI.BeginChangeCheck();
                float val = EditorGUILayout.FloatField(entry.Value, GUILayout.Width(80));
                val = GUILayout.HorizontalSlider(Mathf.Clamp(val, entry.MinValue, entry.MaxValue), entry.MinValue, entry.MaxValue, GUILayout.ExpandWidth(true));
                if (EditorGUI.EndChangeCheck())
                {
                    entry.Value = val;
                    EditingParamTable.SyncFieldsFromEntries();
                }
                GUILayout.Label(entry.Unit, EditorStyles.miniLabel, GUILayout.Width(45));
                GUILayout.Label(entry.Description, EditorStyles.miniLabel, GUILayout.Width(210));
                EditorGUILayout.EndHorizontal();
            }
            EditorGUILayout.EndScrollView();

            EditorGUILayout.BeginHorizontal();
            GUI.backgroundColor = new Color(0.55f, 1f, 0.65f);
            if (GUILayout.Button("파라미터 적용 및 PX4 .params 반영", GUILayout.Height(28)))
            {
                ApplyToVehicle(host.SceneVehicle);
                ExportParamsToDockerConfig(host);
            }
            GUI.backgroundColor = Color.white;
            if (GUILayout.Button("별도 .params 파일 내보내기", GUILayout.Height(28)))
            {
                string script = EditingParamTable.GeneratePx4ParameterScript(cfg, db);
                string path = EditorUtility.SaveFilePanel("Save PX4 Parameter Table", Application.dataPath, $"unity_airframe_{EditingParamTable.Px4SitlModelName}.params", "params");
                if (!string.IsNullOrEmpty(path)) File.WriteAllText(path, script);
            }
            EditorGUILayout.EndHorizontal();
            EditorGUILayout.EndVertical();

            EditorGUILayout.Space(6);

            EditorGUILayout.BeginVertical(EditorStyles.helpBox);
            GUILayout.Label("Step 4.2: PX4 비행 명령 (pxh>) & ROS 2 Offboard", EditorStyles.boldLabel);
            EditorGUILayout.BeginHorizontal();
            if (GUILayout.Button("Arm")) SendPxh(host.Infra, "commander arm");
            if (GUILayout.Button("Disarm")) SendPxh(host.Infra, "commander disarm");
            if (GUILayout.Button("Takeoff")) SendPxh(host.Infra, "commander takeoff");
            if (GUILayout.Button("Land")) SendPxh(host.Infra, "commander land");
            if (GUILayout.Button("Loiter")) SendPxh(host.Infra, "commander mode auto:loiter");
            if (GUILayout.Button("RTL")) SendPxh(host.Infra, "commander mode auto:rtl");
            EditorGUILayout.EndHorizontal();

            bool isVtol = cfg.Category == VehicleAirframeCategory.HybridQuadPlaneVtol || cfg.Category == VehicleAirframeCategory.TiltRotorVtol;
            EditorGUILayout.BeginHorizontal();
            GUI.enabled = isVtol;
            if (GUILayout.Button("VTOL -> Fixed-Wing")) SendPxh(host.Infra, "commander vtol_transition fixedwing");
            if (GUILayout.Button("VTOL -> Multicopter")) SendPxh(host.Infra, "commander vtol_transition multicopter");
            GUI.enabled = true;
            if (GUILayout.Button("Preflight Check")) SendPxh(host.Infra, "commander check");
            if (GUILayout.Button("MAVLink Status")) SendPxh(host.Infra, "mavlink status");
            EditorGUILayout.EndHorizontal();

            EditorGUILayout.BeginHorizontal();
            GUILayout.Label("pxh>", GUILayout.Width(35));
            _customPxhCommand = EditorGUILayout.TextField(_customPxhCommand);
            if (GUILayout.Button("Send", GUILayout.Width(70)) && !string.IsNullOrWhiteSpace(_customPxhCommand))
                SendPxh(host.Infra, _customPxhCommand.Trim());
            EditorGUILayout.EndHorizontal();

            EditorGUILayout.BeginHorizontal();
            _offboardAltitude = EditorGUILayout.FloatField("Offboard Alt (m)", _offboardAltitude);
            _offboardRadius = EditorGUILayout.FloatField("Radius (m)", _offboardRadius);
            if (GUILayout.Button("Start ROS 2 Offboard"))
            {
                var launcher = Object.FindFirstObjectByType<Px4SitlProcessLauncher>();
                if (launcher != null) launcher.StartRos2OffboardNode(_offboardAltitude, _offboardRadius);
                else host.Infra.RunScriptAction($"ros2-offboard --altitude {_offboardAltitude:F1} --radius {_offboardRadius:F1}", "Starting ROS 2 Offboard node...", () => EditorApplication.timeSinceStartup);
            }
            if (GUILayout.Button("Stop Offboard"))
                Object.FindFirstObjectByType<Px4SitlProcessLauncher>()?.StopRos2OffboardNode();
            EditorGUILayout.EndHorizontal();

            var vehicle = host.SceneVehicle;
            if (vehicle != null && vehicle.Engine != null && vehicle.FlightController != null)
            {
                EditorGUILayout.BeginHorizontal();
                if (GUILayout.Button("내부 C# 폐루프 이륙 테스트 (No PX4)"))
                {
                    vehicle.UseInternalFlightController = true;
                    vehicle.SyncAirframeControlParameterTable(resetIfCategoryChanged: false);
                    var st = vehicle.Engine.State;
                    vehicle.FlightController.CommandTakeoff(st.PositionNed, st.AttitudeBodyToNed.ToEulerZyx().Z, 15.0);
                }
                if (GUILayout.Button("PX4 SITL 외부 제어 복귀"))
                {
                    vehicle.FlightController.Disarm();
                    vehicle.UseInternalFlightController = false;
                }
                EditorGUILayout.EndHorizontal();
            }
            EditorGUILayout.EndVertical();
        }

        public void ApplyToVehicle(FlightDynamicsVehicle? vehicle)
        {
            if (EditingParamTable == null) return;
            EditingParamTable.SyncFieldsFromEntries();
            if (vehicle != null)
            {
                Undo.RecordObject(vehicle, "Apply Airframe Parameter Table");
                vehicle.ControlParameterTable = EditingParamTable;
                vehicle.FlightController?.ApplyParameterTable(EditingParamTable);
                EditorUtility.SetDirty(vehicle);
            }
        }

        public void ExportParamsToDockerConfig(FlightDynamicsEngineeringWizard host)
        {
            var launcher = Object.FindFirstObjectByType<Px4SitlProcessLauncher>();
            if (launcher != null)
            {
                launcher.Px4AirframeModel = host.AirframeModel;
                launcher.ExportStage1AirframeParamsForContainer();
            }
            else if (EditingParamTable != null)
            {
                string configDir = Path.Combine(SitlInfraController.GetWorkspaceRoot(), "docker", "config");
                Directory.CreateDirectory(configDir);
                string script = EditingParamTable.GeneratePx4ParameterScript(host.GetCurrentConfig(), host.GetCurrentBakedDb());
                File.WriteAllText(Path.Combine(configDir, $"unity_airframe_{EditingParamTable.Px4SitlModelName}.params"), script);
                File.WriteAllText(Path.Combine(configDir, "unity_airframe.params"), script);
            }
        }

        private static void SendPxh(SitlInfraController infra, string command)
        {
            var launcher = Object.FindFirstObjectByType<Px4SitlProcessLauncher>();
            if (launcher != null) launcher.SendPxhCommand(command);
            else infra.RunScriptAction($"pxh {command}", $"[pxh>] {command}", () => EditorApplication.timeSinceStartup);
        }
    }
}

