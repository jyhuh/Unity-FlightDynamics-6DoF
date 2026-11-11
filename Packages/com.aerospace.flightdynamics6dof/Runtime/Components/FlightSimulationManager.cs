// Licensed under the Apache License, Version 2.0.
#nullable enable
using System;
using System.Collections.Generic;
using UnityEngine;
using FlightDynamics6DoF.Core.Dynamics;
using FlightDynamics6DoF.Core.Gnc;
using FlightDynamics6DoF.Core.Math;
using FlightDynamics6DoF.Core.Mavlink;
using FlightDynamics6DoF.Core.Specs;

namespace FlightDynamics6DoF.Unity
{
    /// <summary>
    /// Scene-level Flight Dynamics Environment, PX4 SITL Process Coordinator &amp; Interactive Telemetry HUD.
    /// Unity + FlightDynamics6DoF.Core.dll acts strictly as the 6-DOF Physics, Multi-Fidelity Aerodynamics &amp; Sensor Environment,
    /// while the independent external PX4 SITL process controls the vehicle over TCP 4560 (HIL_SENSOR / HIL_ACTUATOR_CONTROLS).
    /// </summary>
    [DisallowMultipleComponent]
    public class FlightSimulationManager : MonoBehaviour
    {
        [Header("Primary Environment Components")]
        public FlightDynamicsVehicle? PrimaryVehicle;
        public MavlinkNetworkNode? PrimaryMavlinkNode;
        public Px4SitlProcessLauncher? Px4Launcher;
        public FlightCameraController? CameraController;

        [Header("WGS84 Geodetic Origin (NED Reference 0,0,0)")]
        public double OriginLatitudeDeg = 37.5665;
        public double OriginLongitudeDeg = 126.9780;
        public double OriginAltitudeMsl = 50.0;

        [Header("Atmospheric & Wind Environment (NED)")]
        public Vector3 MeanWindNedMetersPerSec = Vector3.zero;
        public bool ShowEngineeringHud = true;

        private bool _guiMavConnectedLatch;
        private bool _guiPx4RunningLatch;
        private Vector2 _hudScrollPos;
        private string _customPxhCommand = "commander takeoff";

        public void SwitchVehicleAndPx4Model(VehicleAirframeCategory category)
        {
            SwitchVehiclePresetLive(category);

            if (Px4Launcher != null)
            {
                string targetModel = category switch
                {
                    VehicleAirframeCategory.FixedWing => "plane",
                    VehicleAirframeCategory.HybridQuadPlaneVtol => "standard_vtol",
                    _ => "iris"
                };

                Px4Launcher.SwitchAirframeModelAndRestart(targetModel);
            }
        }

        [RuntimeInitializeOnLoadMethod(RuntimeInitializeLoadType.AfterSceneLoad)]
        private static void AutoBootstrapOnPlayIfSceneEmpty()
        {
            var existingVehicle = FindFirstObjectByType<FlightDynamicsVehicle>();
            if (existingVehicle == null)
            {
                Debug.Log("[FlightDynamics6DoF] Empty scene detected on Play — spawning 3D Environment for PX4 SITL (TCP 4560)...");
                SpawnCompleteSimulationEnvironment(
                    VehicleAirframeCategory.Multirotor,
                    SimulationFidelityLevel.Level2_6DoFBladeAndStrip,
                    MavlinkBridgeMode.ExternalSitlHil,
                    autoTakeoff: false);
            }
            else
            {
                if (existingVehicle.transform.Find("VisualAirframeRoot") == null && existingVehicle.VisualRotorTransforms.Count == 0)
                    ProceduralAircraftVisualBuilder.RebuildVehicleVisuals(existingVehicle);
                ProceduralAircraftVisualBuilder.EnsureGroundEnvironment();
                EnsureCameraController(existingVehicle);
            }
        }

        public static FlightSimulationManager SpawnCompleteSimulationEnvironment(
            VehicleAirframeCategory presetCategory,
            SimulationFidelityLevel fidelityLevel,
            MavlinkBridgeMode mavlinkMode = MavlinkBridgeMode.ExternalSitlHil,
            bool autoTakeoff = false)
        {
            // Match PX4 SimulatorMavlink 250 Hz (4000 us) IMU lockstep clock and keep running when QGC is focused
            Application.runInBackground = true;
            Time.fixedDeltaTime = 0.004f;

            // 1. Ground Environment & Helipad
            ProceduralAircraftVisualBuilder.EnsureGroundEnvironment();

            // 2. Ensure Directional Light exists
            if (FindFirstObjectByType<Light>() == null)
            {
                GameObject lightGo = new GameObject("Directional Light");
                Light light = lightGo.AddComponent<Light>();
                light.type = LightType.Directional;
                light.intensity = 1.2f;
                lightGo.transform.rotation = Quaternion.Euler(50f, -30f, 0f);
            }

            // 3. Vehicle GameObject + 3D Procedural Airframe
            FlightDynamicsVehicle? vehicle = FindFirstObjectByType<FlightDynamicsVehicle>();
            if (vehicle == null)
            {
                GameObject vehicleGo = new GameObject("FlightDynamicsVehicle_UAV");
                vehicleGo.transform.position = Vector3.zero;
                vehicle = vehicleGo.AddComponent<FlightDynamicsVehicle>();
            }

            vehicle.InlineConfig = presetCategory switch
            {
                VehicleAirframeCategory.FixedWing => AircraftConfigSpec.CreateFixedWingUav(),
                VehicleAirframeCategory.HybridQuadPlaneVtol => AircraftConfigSpec.CreateQuadPlaneVtolMillie(),
                _ => AircraftConfigSpec.CreateStandardQuadcopterX()
            };
            vehicle.FidelityLevel = fidelityLevel;
            vehicle.UseInternalFlightController = (mavlinkMode == MavlinkBridgeMode.DirectGcsAutopilot);
            vehicle.AutoArmAndTakeoffOnStart = autoTakeoff;
            vehicle.InitialTakeoffAltitudeMeters = 10.0f;
            vehicle.RunStage1AnalysisAndBake();
            ProceduralAircraftVisualBuilder.RebuildVehicleVisuals(vehicle);
            vehicle.InitializeSimulationEngine();

            // 4. MAVLink Network Node (Default: External PX4 SITL TCP Server on 4560)
            MavlinkNetworkNode? mavNode = vehicle.GetComponent<MavlinkNetworkNode>();
            if (mavNode == null)
                mavNode = vehicle.gameObject.AddComponent<MavlinkNetworkNode>();
            mavNode.Vehicle = vehicle;
            mavNode.BridgeMode = mavlinkMode;
            mavNode.Transport = mavlinkMode == MavlinkBridgeMode.ExternalSitlHil
                ? NetworkTransportType.TcpPx4SitlServer
                : NetworkTransportType.UdpGcsOrSitl;

            // 5. Simulation Manager + PX4 SITL Process Launcher
            FlightSimulationManager? mgr = FindFirstObjectByType<FlightSimulationManager>();
            if (mgr == null)
            {
                GameObject mgrGo = new GameObject("FlightSimulationManager");
                mgr = mgrGo.AddComponent<FlightSimulationManager>();
            }

            Px4SitlProcessLauncher? px4Launcher = mgr.GetComponent<Px4SitlProcessLauncher>();
            if (px4Launcher == null)
                px4Launcher = mgr.gameObject.AddComponent<Px4SitlProcessLauncher>();
            px4Launcher.TargetVehicle = vehicle;

            mgr.PrimaryVehicle = vehicle;
            mgr.PrimaryMavlinkNode = mavNode;
            mgr.Px4Launcher = px4Launcher;
            mgr.CameraController = EnsureCameraController(vehicle);

            return mgr;
        }

        public static FlightCameraController EnsureCameraController(FlightDynamicsVehicle vehicle)
        {
            Camera? cam = Camera.main;
            if (cam == null)
                cam = FindFirstObjectByType<Camera>();
            if (cam == null)
            {
                GameObject camGo = new GameObject("Main Camera");
                camGo.tag = "MainCamera";
                cam = camGo.AddComponent<Camera>();
            }

            FlightCameraController? ctrl = cam.GetComponent<FlightCameraController>();
            if (ctrl == null)
                ctrl = cam.gameObject.AddComponent<FlightCameraController>();
            ctrl.TargetVehicle = vehicle;
            return ctrl;
        }

        private void Start()
        {
            Application.runInBackground = true;
            Time.fixedDeltaTime = 0.004f;
            if (PrimaryVehicle == null)
                PrimaryVehicle = FindFirstObjectByType<FlightDynamicsVehicle>();
            if (PrimaryMavlinkNode == null)
                PrimaryMavlinkNode = FindFirstObjectByType<MavlinkNetworkNode>();
            if (Px4Launcher == null)
                Px4Launcher = GetComponent<Px4SitlProcessLauncher>() ?? gameObject.AddComponent<Px4SitlProcessLauncher>();
            if (PrimaryVehicle != null)
            {
                Px4Launcher.TargetVehicle = PrimaryVehicle;
                if (CameraController == null)
                    CameraController = EnsureCameraController(PrimaryVehicle);
            }

            SyncEnvironmentToVehicle();
        }

        private void FixedUpdate()
        {
            SyncEnvironmentToVehicle();
        }

        private void SyncEnvironmentToVehicle()
        {
            if (PrimaryVehicle == null || PrimaryVehicle.Engine == null)
                return;

            var eng = PrimaryVehicle.Engine;
            eng.OriginLatitudeDeg = OriginLatitudeDeg;
            eng.OriginLongitudeDeg = OriginLongitudeDeg;
            eng.OriginAltitudeMsl = OriginAltitudeMsl;
            eng.MeanWindNed = new Vec3d(MeanWindNedMetersPerSec.x, MeanWindNedMetersPerSec.y, MeanWindNedMetersPerSec.z);
        }

        public void SwitchVehiclePresetLive(VehicleAirframeCategory category)
        {
            if (PrimaryVehicle == null) return;
            PrimaryVehicle.ProfileAsset = null;
            PrimaryVehicle.InlineConfig = category switch
            {
                VehicleAirframeCategory.FixedWing => AircraftConfigSpec.CreateFixedWingUav(),
                VehicleAirframeCategory.HybridQuadPlaneVtol => AircraftConfigSpec.CreateQuadPlaneVtolMillie(),
                _ => AircraftConfigSpec.CreateStandardQuadcopterX()
            };
            PrimaryVehicle.RunStage1AnalysisAndBake();
            ProceduralAircraftVisualBuilder.RebuildVehicleVisuals(PrimaryVehicle);

            if (PrimaryVehicle.Engine != null)
                PrimaryVehicle.Engine.ResetState(Vec3d.Zero, Quatd.Identity);
        }

        private void OnGUI()
        {
            if (!ShowEngineeringHud || PrimaryVehicle == null || PrimaryVehicle.Engine == null)
                return;

            if (Event.current.type == EventType.Layout)
            {
                _guiMavConnectedLatch = PrimaryMavlinkNode != null && PrimaryMavlinkNode.IsConnected;
                _guiPx4RunningLatch = Px4Launcher != null && Px4Launcher.IsPx4ProcessRunning;
            }

            var eng = PrimaryVehicle.Engine;
            var st = eng.State;
            var db = PrimaryVehicle.ActiveBakedDatabase;
            Vec3d eulerDeg = st.AttitudeBodyToNed.ToEulerZyx() * FrameConverter.Rad2Deg;

            GUILayout.BeginArea(new Rect(14, 14, 530, Mathf.Min(780, Screen.height - 28)), GUI.skin.box);
            _hudScrollPos = GUILayout.BeginScrollView(_hudScrollPos);

            GUILayout.Label($"<b>FlightDynamics 6-DOF Environment & PX4 SITL Bridge</b>");
            GUILayout.Label($"Vehicle Plant: <b>{PrimaryVehicle.ActiveConfig.AircraftName}</b> | Sim Time: {st.SimTimeSec:F2} s");
            GUILayout.Space(4);

            // 1. PX4 SITL External Control Section (Primary)
            GUILayout.Label($"<b>[1. Independent PX4 SITL Controller (TCP 4560 HIL Lockstep)]</b>");
            if (PrimaryMavlinkNode != null)
            {
                var br = PrimaryMavlinkNode.Bridge;
                string connStr = _guiMavConnectedLatch ? "<color=#44ff66>CONNECTED</color>" : "<color=#ffcc33>LISTENING ON TCP :4560</color>";
                GUILayout.Label($"TCP 4560 Status: {connStr} | PX4 Armed: <b>{br.IsExternalSitlArmed}</b>");
                GUILayout.Label($"HIL_SENSOR Sent: {br.TotalFramesSent} | HIL_ACTUATOR_CONTROLS Recv: {br.TotalHilActuatorFramesReceived}");
                GUILayout.Label($"PX4 Motor Controls [0..3]: [{st.ActuatorControlsNormalized[0]:F2}, {st.ActuatorControlsNormalized[1]:F2}, {st.ActuatorControlsNormalized[2]:F2}, {st.ActuatorControlsNormalized[3]:F2}]");
            }

            if (Px4Launcher != null)
            {
                GUILayout.Label($"Container ({Px4Launcher.RuntimeBackend}): <i>{Px4Launcher.LastConsoleSummary}</i>");
                GUILayout.BeginHorizontal();
                if (!_guiPx4RunningLatch)
                {
                    if (GUILayout.Button("Start PX4 + ROS2"))
                        Px4Launcher.StartPx4SitlProcess();
                }
                else
                {
                    if (GUILayout.Button("Stop Containers"))
                        Px4Launcher.StopPx4SitlProcess();
                }

                if (GUILayout.Button("Takeoff"))
                    Px4Launcher.SendPxhCommand("commander takeoff");
                if (GUILayout.Button("Loiter"))
                    Px4Launcher.SendPxhCommand("commander mode auto:loiter");
                if (GUILayout.Button("RTL"))
                    Px4Launcher.SendPxhCommand("commander mode auto:rtl");
                if (GUILayout.Button("Land"))
                    Px4Launcher.SendPxhCommand("commander land");
                GUILayout.EndHorizontal();

                // VTOL Specific Controls
                if (PrimaryVehicle.ActiveConfig.Category == VehicleAirframeCategory.HybridQuadPlaneVtol ||
                    Px4Launcher.Px4AirframeModel.Contains("vtol"))
                {
                    GUILayout.BeginHorizontal();
                    GUILayout.Label("VTOL:", GUILayout.Width(45));
                    if (GUILayout.Button("Transition to Fixed-Wing (FW)"))
                        Px4Launcher.SendPxhCommand("commander vtol_transition fixedwing");
                    if (GUILayout.Button("Transition to Hover (MC)"))
                        Px4Launcher.SendPxhCommand("commander vtol_transition multicopter");
                    GUILayout.EndHorizontal();
                }

                // Interactive pxh Shell Input Field (Replaces CLI Terminal)
                GUILayout.BeginHorizontal();
                GUILayout.Label("pxh>", GUILayout.Width(38));
                _customPxhCommand = GUILayout.TextField(_customPxhCommand);
                if (GUILayout.Button("Send", GUILayout.Width(55)))
                    Px4Launcher.SendPxhCommand(_customPxhCommand);
                GUILayout.EndHorizontal();

                GUILayout.BeginHorizontal();
                if (GUILayout.Button("Arm"))
                    Px4Launcher.SendPxhCommand("commander arm");
                if (GUILayout.Button("Disarm"))
                    Px4Launcher.SendPxhCommand("commander disarm");
                if (GUILayout.Button("Status"))
                    Px4Launcher.SendPxhCommand("commander check");
                if (GUILayout.Button("MAVLink Status"))
                    Px4Launcher.SendPxhCommand("mavlink status");
                GUILayout.EndHorizontal();

                GUILayout.BeginHorizontal();
                if (GUILayout.Button("ROS 2: Start Offboard Orbit (10m)"))
                    Px4Launcher.StartRos2OffboardNode(10.0f, 8.0f);
                if (GUILayout.Button("ROS 2: Stop Offboard Node"))
                    Px4Launcher.StopRos2OffboardNode();
                GUILayout.EndHorizontal();
            }

            GUILayout.Space(4);
            GUILayout.Label($"<b>[2. Airframe & PX4 Model One-Click Switch]</b>");
            GUILayout.BeginHorizontal();
            if (GUILayout.Button("Quad X-450 (iris)"))
                SwitchVehicleAndPx4Model(VehicleAirframeCategory.Multirotor);
            if (GUILayout.Button("Fixed-Wing (plane)"))
                SwitchVehicleAndPx4Model(VehicleAirframeCategory.FixedWing);
            if (GUILayout.Button("QuadPlane (standard_vtol)"))
                SwitchVehicleAndPx4Model(VehicleAirframeCategory.HybridQuadPlaneVtol);
            GUILayout.EndHorizontal();

            GUILayout.BeginHorizontal();
            if (GUILayout.Button("Lv0: Rigidbody"))
                PrimaryVehicle.FidelityLevel = SimulationFidelityLevel.Level0_UnityRigidbody;
            if (GUILayout.Button("Lv1: Surrogate"))
                PrimaryVehicle.FidelityLevel = SimulationFidelityLevel.Level1_6DoFSurrogate;
            if (GUILayout.Button("Lv2: BEMT+Strip"))
                PrimaryVehicle.FidelityLevel = SimulationFidelityLevel.Level2_6DoFBladeAndStrip;
            if (GUILayout.Button("Lv3: Full VLM"))
                PrimaryVehicle.FidelityLevel = SimulationFidelityLevel.Level3_6DoFFullAeroWake;
            GUILayout.EndHorizontal();

            if (CameraController != null)
            {
                GUILayout.BeginHorizontal();
                GUILayout.Label("Camera:", GUILayout.Width(60));
                if (GUILayout.Button("Chase Orbit")) CameraController.ViewMode = CameraViewMode.ChaseOrbit;
                if (GUILayout.Button("Close Follow")) CameraController.ViewMode = CameraViewMode.CloseFollow;
                if (GUILayout.Button("FPV Nose")) CameraController.ViewMode = CameraViewMode.FpvNoseCam;
                if (GUILayout.Button("Top-Down")) CameraController.ViewMode = CameraViewMode.TopDownMap;
                GUILayout.EndHorizontal();
            }

            GUILayout.Space(4);
            GUILayout.Label($"<b>[3. 6-DOF Environment State (Aerospace NED / FRD)]</b>");
            GUILayout.Label($"Pos NED [m]: N={st.PositionNed.X:F2}, E={st.PositionNed.Y:F2}, D={st.PositionNed.Z:F2} (<b>AGL: {st.AltitudeAglMeters:F2} m</b>)");
            GUILayout.Label($"Vel FRD [m/s]: u={st.VelocityBodyFrd.X:F2}, v={st.VelocityBodyFrd.Y:F2}, w={st.VelocityBodyFrd.Z:F2} (<b>V={st.AirspeedMps:F2} m/s</b>)");
            GUILayout.Label($"Attitude [deg]: Roll={eulerDeg.X:F1}, Pitch={eulerDeg.Y:F1}, Yaw={eulerDeg.Z:F1}");
            GUILayout.Label($"Rates [deg/s]: p={st.AngularVelocityBodyFrd.X * FrameConverter.Rad2Deg:F1}, q={st.AngularVelocityBodyFrd.Y * FrameConverter.Rad2Deg:F1}, r={st.AngularVelocityBodyFrd.Z * FrameConverter.Rad2Deg:F1}");
            GUILayout.Label($"Aero Angles: Alpha={st.AlphaRad * FrameConverter.Rad2Deg:F2} deg | Beta={st.BetaRad * FrameConverter.Rad2Deg:F2} deg");

            GUILayout.Space(4);
            GUILayout.Label($"<b>[4. Stage-1 Baked Aerodynamic & Stability Report]</b>");
            GUILayout.Label($"Static Margin: <b>{(db.StaticMarginFraction * 100.0):F1}% MAC</b> | Neutral Point x_NP: {db.NeutralPointXFrd:F3} m");
            GUILayout.Label($"Derivatives: CL_a={db.CLAlpha:F2}/rad, Cm_a={db.CmAlpha:F3}/rad, Cn_b={db.CnBeta:F3}/rad, CD0={db.CD0:F4}");

            // Optional offline fallback test toggle when PX4 is not running
            if (PrimaryMavlinkNode != null && !_guiMavConnectedLatch && PrimaryVehicle.FlightController != null)
            {
                GUILayout.Space(4);
                GUILayout.BeginHorizontal();
                GUILayout.Label("No PX4 TCP 4560 yet — Offline Physics Test:");
                if (GUILayout.Button("Test Physics Takeoff (No PX4)"))
                {
                    PrimaryMavlinkNode.BridgeMode = MavlinkBridgeMode.DirectGcsAutopilot;
                    PrimaryVehicle.UseInternalFlightController = true;
                    PrimaryVehicle.FlightController.CommandTakeoff(st.PositionNed, st.AttitudeBodyToNed.ToEulerZyx().Z, 10.0);
                }
                if (PrimaryVehicle.UseInternalFlightController && GUILayout.Button("Return to PX4 SITL Mode"))
                {
                    PrimaryVehicle.FlightController.Disarm();
                    PrimaryVehicle.UseInternalFlightController = false;
                    PrimaryMavlinkNode.BridgeMode = MavlinkBridgeMode.ExternalSitlHil;
                }
                GUILayout.EndHorizontal();
            }

            GUILayout.EndScrollView();
            GUILayout.EndArea();
        }
    }
}
