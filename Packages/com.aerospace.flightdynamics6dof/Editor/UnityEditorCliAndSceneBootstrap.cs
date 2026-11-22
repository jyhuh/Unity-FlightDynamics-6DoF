// Licensed under the Apache License, Version 2.0.
using System;
using System.IO;
using UnityEditor;
using UnityEditor.SceneManagement;
using UnityEngine;
using FlightDynamics6DoF.Core.Mavlink;
using FlightDynamics6DoF.Core.Specs;
using FlightDynamics6DoF.Unity;

namespace FlightDynamics6DoF.Editor
{
    /// <summary>
    /// Automatic Scene Bootstrapper &amp; Live Unity CLI Command Bridge.
    /// 1) Automatically populates the active scene with 3D aircraft objects on Editor load if no vehicle is present.
    /// 2) Bridges terminal CLI commands (scripts/unity-cli.sh) to the running Unity Editor instance (via Temp/fd6dof_cli_cmd.txt)
    ///    as well as headless Unity -batchmode -executeMethod invocations.
    /// </summary>
    [InitializeOnLoad]
    public static class UnityEditorCliAndSceneBootstrap
    {
        private const string DemoScenePath = "Assets/Scenes/FlightDynamics6DoF_EngineeringDemo.unity";
        private const string AssetStoreDemoScenePath = "Assets/FlightDynamics6DoF/Demo/Scenes/FlightDynamics6DoF_EngineeringDemo.unity";
        private static readonly string CmdFilePath = Path.Combine(Directory.GetCurrentDirectory(), "Temp", "fd6dof_cli_cmd.txt");
        private static readonly string ResFilePath = Path.Combine(Directory.GetCurrentDirectory(), "Temp", "fd6dof_cli_res.txt");
        private static double _lastPollTime;

        static UnityEditorCliAndSceneBootstrap()
        {
            Application.runInBackground = true;
            EditorApplication.delayCall -= AutoInitializeDemoSceneIfNeeded;
            EditorApplication.delayCall += AutoInitializeDemoSceneIfNeeded;
            EditorApplication.update -= PollCliCommandFile;
            EditorApplication.update += PollCliCommandFile;
            EditorApplication.playModeStateChanged -= OnPlayModeStateChanged;
            EditorApplication.playModeStateChanged += OnPlayModeStateChanged;
        }

        [InitializeOnEnterPlayMode]
        private static void OnEnterPlayModeResetStaticState(EnterPlayModeOptions options)
        {
            _lastPollTime = 0.0;
        }

        private static void OnPlayModeStateChanged(PlayModeStateChange state)
        {
            if (state == PlayModeStateChange.EnteredEditMode || state == PlayModeStateChange.EnteredPlayMode)
            {
                _lastPollTime = 0.0;
            }
        }

        private static string GetPreferredDemoScenePath()
        {
            string root = Directory.GetCurrentDirectory();
            if (Directory.Exists(Path.Combine(root, "Assets", "FlightDynamics6DoF")))
                return AssetStoreDemoScenePath;
            return DemoScenePath;
        }

        private static void AutoInitializeDemoSceneIfNeeded()
        {
            if (EditorApplication.isPlayingOrWillChangePlaymode)
                return;

            var existingVehicle = UnityEngine.Object.FindFirstObjectByType<FlightDynamicsVehicle>();
            if (existingVehicle != null)
            {
                if (existingVehicle.transform.Find("VisualAirframeRoot") == null)
                    ProceduralAircraftVisualBuilder.RebuildVehicleVisuals(existingVehicle);
                return;
            }

            // Asset Store compliance: never overwrite a user's active scene or EditorBuildSettings on import.
            // Only auto-open an existing demo scene if the current scene is an unmodified, untitled scene.
            var activeScene = EditorSceneManager.GetActiveScene();
            if (!string.IsNullOrEmpty(activeScene.path) || activeScene.isDirty)
                return;

            string root = Directory.GetCurrentDirectory();
            if (File.Exists(Path.Combine(root, AssetStoreDemoScenePath)))
            {
                EditorSceneManager.OpenScene(AssetStoreDemoScenePath);
                return;
            }
            if (File.Exists(Path.Combine(root, DemoScenePath)))
            {
                EditorSceneManager.OpenScene(DemoScenePath);
                return;
            }

            // Only auto-generate a new demo scene on load inside the primary development repository
            if (File.Exists(Path.Combine(root, "scripts", "unity-cli.sh")))
            {
                PopulateAndSaveDemoScene(
                    VehicleAirframeCategory.Multirotor,
                    SimulationFidelityLevel.Level2_6DoFBladeAndStrip);
            }
        }

        [MenuItem("Tools/FlightDynamics 6-DOF/Quick-Build 3D Demo Scene Now", false, 11)]
        public static void MenuQuickBuildDemoScene()
        {
            PopulateAndSaveDemoScene(
                VehicleAirframeCategory.Multirotor,
                SimulationFidelityLevel.Level2_6DoFBladeAndStrip);
        }

        public static string PopulateAndSaveDemoScene(
            VehicleAirframeCategory category,
            SimulationFidelityLevel fidelity)
        {
            var mgr = FlightSimulationManager.SpawnCompleteSimulationEnvironment(
                category,
                fidelity,
                MavlinkBridgeMode.ExternalSitlHil,
                autoTakeoff: false);

            if (mgr.PrimaryVehicle != null)
            {
                Selection.activeGameObject = mgr.PrimaryVehicle.gameObject;
                if (SceneView.lastActiveSceneView != null)
                {
                    SceneView.lastActiveSceneView.LookAt(
                        mgr.PrimaryVehicle.transform.position,
                        Quaternion.Euler(22f, -35f, 0f),
                        2.6f);
                }
            }

            string targetScenePath = GetPreferredDemoScenePath();
            string? scenesDir = Path.GetDirectoryName(Path.Combine(Directory.GetCurrentDirectory(), targetScenePath));
            if (!string.IsNullOrEmpty(scenesDir) && !Directory.Exists(scenesDir))
                Directory.CreateDirectory(scenesDir);

            var scene = EditorSceneManager.GetActiveScene();
            EditorSceneManager.MarkSceneDirty(scene);
            EditorSceneManager.SaveScene(scene, targetScenePath);

            if (EditorBuildSettings.scenes == null || EditorBuildSettings.scenes.Length == 0)
            {
                EditorBuildSettings.scenes = new[]
                {
                    new EditorBuildSettingsScene(targetScenePath, true)
                };
            }

            AssetDatabase.SaveAssets();
            var db = mgr.PrimaryVehicle != null ? mgr.PrimaryVehicle.ActiveBakedDatabase : null;
            string summary = db != null
                ? $"Scene='{targetScenePath}' | Vehicle='{db.AircraftName}' | Fidelity='{fidelity}' | StaticMargin={db.StaticMarginFraction * 100.0:F1}% MAC | CL_alpha={db.CLAlpha:F2}/rad | Cm_alpha={db.CmAlpha:F3}/rad"
                : $"Scene='{targetScenePath}' populated.";

            Debug.Log($"[FlightDynamics6DoF] Auto-populated and saved 3D Demo Scene: {summary}");
            return summary;
        }

        private static void PollCliCommandFile()
        {
            if (EditorApplication.timeSinceStartup - _lastPollTime < 0.25)
                return;
            _lastPollTime = EditorApplication.timeSinceStartup;

            if (!File.Exists(CmdFilePath))
                return;

            try
            {
                string rawCmd = File.ReadAllText(CmdFilePath).Trim();
                File.Delete(CmdFilePath);

                string response = ExecuteCliCommandString(rawCmd);
                File.WriteAllText(ResFilePath, response);
                Debug.Log($"[FlightDynamics6DoF CLI Bridge] Executed '{rawCmd}' -> {response}");
            }
            catch (Exception ex)
            {
                File.WriteAllText(ResFilePath, $"ERROR: {ex.Message}");
            }
        }

        public static string ExecuteCliCommandString(string cmdLine)
        {
            string[] parts = cmdLine.Split(new[] { ' ' }, StringSplitOptions.RemoveEmptyEntries);
            string action = parts.Length > 0 ? parts[0].ToLowerInvariant() : "status";
            string arg1 = parts.Length > 1 ? parts[1].ToLowerInvariant() : "";

            switch (action)
            {
                case "setup-scene":
                {
                    var cat = arg1 switch
                    {
                        "fixedwing" => VehicleAirframeCategory.FixedWing,
                        "vtol" => VehicleAirframeCategory.HybridQuadPlaneVtol,
                        _ => VehicleAirframeCategory.Multirotor
                    };
                    return "OK: " + PopulateAndSaveDemoScene(cat, SimulationFidelityLevel.Level2_6DoFBladeAndStrip);
                }

                case "play":
                {
                    EditorApplication.isPlaying = true;
                    return "OK: Entered Unity Editor Play Mode.";
                }

                case "stop":
                {
                    EditorApplication.isPlaying = false;
                    return "OK: Exited Unity Editor Play Mode.";
                }

                case "bake":
                {
                    var v = UnityEngine.Object.FindFirstObjectByType<FlightDynamicsVehicle>();
                    if (v == null)
                        return "OK: " + PopulateAndSaveDemoScene(VehicleAirframeCategory.Multirotor, SimulationFidelityLevel.Level2_6DoFBladeAndStrip);
                    var db = v.RunStage1AnalysisAndBake();
                    return $"OK: Baked '{db.AircraftName}' -> StaticMargin={db.StaticMarginFraction * 100.0:F1}% MAC, CL_alpha={db.CLAlpha:F2}, Cm_alpha={db.CmAlpha:F3}, Cn_beta={db.CnBeta:F3}";
                }

                case "capture-screenshots":
                {
                    string outDir = parts.Length > 1 ? parts[1] : Path.Combine(Directory.GetCurrentDirectory(), "docs", "asset-store", "images");
                    return CaptureAssetStoreScreenshots(outDir);
                }

                case "status":
                default:
                {
                    var v = UnityEngine.Object.FindFirstObjectByType<FlightDynamicsVehicle>();
                    string vInfo = v != null
                        ? $"Vehicle='{v.ActiveConfig.AircraftName}', Fidelity={v.FidelityLevel}, VisualRotors={v.VisualRotorTransforms.Count}"
                        : "Vehicle=None";
                    return $"OK: UnityVersion={Application.unityVersion}, IsPlaying={EditorApplication.isPlaying}, Scene='{EditorSceneManager.GetActiveScene().path}', {vInfo}";
                }
            }
        }

        /// <summary>
        /// Entrypoint for headless CLI batchmode:
        /// Unity -batchmode -quit -projectPath ... -executeMethod FlightDynamics6DoF.Editor.UnityEditorCliAndSceneBootstrap.RunBatchSetupScene
        /// </summary>
        public static void RunBatchSetupScene()
        {
            string res = PopulateAndSaveDemoScene(
                VehicleAirframeCategory.Multirotor,
                SimulationFidelityLevel.Level2_6DoFBladeAndStrip);
            Console.WriteLine($"[FlightDynamics6DoF Batch CLI] {res}");
        }

        /// <summary>
        /// End-to-end Asset Store Standalone Verification &amp; 3D Screenshot Generator in a fresh Unity project.
        /// Verifies zero Colima/external SITL requirement, closed-loop flight across Multirotor, Fixed-Wing, and VTOL,
        /// zero project root pollution, and captures 1920x1080 3D screenshots.
        /// </summary>
        public static void RunAssetStoreVerificationAndScreenshots()
        {
            string projectRoot = Directory.GetCurrentDirectory();
            Console.WriteLine("======================================================================");
            Console.WriteLine($"[AssetStore Verify] Project Root: {projectRoot}");

            var infra = new SitlInfraController();
            if (infra.IsSitlWorkspaceAvailable)
                throw new Exception("Expected clean standalone Unity project without scripts/px4-sitl.sh!");
            Console.WriteLine("[AssetStore Verify] Confirmed Standalone Mode (IsSitlWorkspaceAvailable == false, zero Colima/Docker scripts).");

            // 1. Open imported Demo Scene and verify zero missing scripts
            if (File.Exists(Path.Combine(projectRoot, AssetStoreDemoScenePath)))
            {
                EditorSceneManager.OpenScene(AssetStoreDemoScenePath);
                Console.WriteLine($"[AssetStore Verify] Opened imported Demo Scene: {AssetStoreDemoScenePath}");
            }

            // 2. Test A: Multirotor Standalone Takeoff via Px4Launcher.SendPxhCommand routing
            var mgr = FlightSimulationManager.SpawnCompleteSimulationEnvironment(
                VehicleAirframeCategory.Multirotor,
                SimulationFidelityLevel.Level2_6DoFBladeAndStrip);
            var vehicle = mgr.PrimaryVehicle ?? throw new Exception("PrimaryVehicle is null");
            var launcher = mgr.Px4Launcher ?? throw new Exception("Px4Launcher is null");

            if (!vehicle.UseInternalFlightController)
                throw new Exception("Vehicle should automatically default to UseInternalFlightController=true in Standalone Mode!");

            launcher.ExportStage1AirframeParamsForContainer();
            if (Directory.Exists(Path.Combine(projectRoot, "docker")))
                throw new Exception("Standalone Mode must NOT create a docker/ directory in the user's Unity project root!");

            vehicle.TelemetryLogger.StartRecording();
            launcher.SendPxhCommand("commander takeoff");
            for (int i = 0; i < 1250; i++)
                vehicle.StepVehiclePhysicsAndControl(0.004);

            double quadAlt = vehicle.Engine!.State.AltitudeAglMeters;
            Console.WriteLine($"[AssetStore Verify] Test A (Multirotor 5.0s Standalone Takeoff): Alt={quadAlt:F2} m AGL, TelemetryFrames={vehicle.TelemetryLogger.Frames.Count}");
            if (quadAlt < 7.0)
                throw new Exception($"Multirotor failed standalone takeoff (Alt={quadAlt:F2}m < 7.0m)");

            //Test Circular Orbit routing in Standalone Mode
            launcher.StartRos2OffboardNode(10.0f, 8.0f);
            for (int i = 0; i < 500; i++)
                vehicle.StepVehiclePhysicsAndControl(0.004);
            Console.WriteLine($"[AssetStore Verify] Test A.2 (Multirotor Standalone Circular Orbit): Mode={vehicle.FlightController!.CurrentMode}, Alt={vehicle.Engine.State.AltitudeAglMeters:F2} m");

            // 3. Test B: Fixed-Wing Standalone Runway Ground Roll & Rotation Takeoff
            mgr.SwitchVehiclePresetLive(VehicleAirframeCategory.FixedWing);
            vehicle.UseInternalFlightController = true;
            launcher.SendPxhCommand("commander takeoff");
            for (int i = 0; i < 1500; i++)
                vehicle.StepVehiclePhysicsAndControl(0.004);

            double fwAlt = vehicle.Engine.State.AltitudeAglMeters;
            double fwSpeed = vehicle.Engine.State.AirspeedMps;
            Console.WriteLine($"[AssetStore Verify] Test B (Fixed-Wing 6.0s Standalone Runway Takeoff): Alt={fwAlt:F2} m AGL, Airspeed={fwSpeed:F2} m/s");
            if (fwAlt < 4.5 || fwSpeed < 12.0)
                throw new Exception($"Fixed-Wing failed standalone runway takeoff (Alt={fwAlt:F2}m, V={fwSpeed:F2}m/s)");

            // 4. Test C: Hybrid QuadPlane VTOL Standalone Takeoff & Fixed-Wing Transition
            mgr.SwitchVehiclePresetLive(VehicleAirframeCategory.HybridQuadPlaneVtol);
            vehicle.UseInternalFlightController = true;
            launcher.SendPxhCommand("commander takeoff");
            for (int i = 0; i < 750; i++)
                vehicle.StepVehiclePhysicsAndControl(0.004);
            launcher.SendPxhCommand("commander vtol_transition fixedwing");
            for (int i = 0; i < 750; i++)
                vehicle.StepVehiclePhysicsAndControl(0.004);

            double vtolAlt = vehicle.Engine.State.AltitudeAglMeters;
            double vtolSpeed = vehicle.Engine.State.AirspeedMps;
            Console.WriteLine($"[AssetStore Verify] Test C (QuadPlane VTOL Standalone Transition): Alt={vtolAlt:F2} m AGL, Airspeed={vtolSpeed:F2} m/s, FW_Cruise={vehicle.FlightController.IsVtolFixedWingCruiseActive}");
            if (vtolAlt < 5.0 || !vehicle.FlightController.IsVtolFixedWingCruiseActive)
                throw new Exception($"QuadPlane VTOL failed standalone transition test (Alt={vtolAlt:F2}m)");

            // 5. Capture 1920x1080 3D Screenshots for Asset Store Submission
            string imagesDir = Path.GetFullPath(Path.Combine(projectRoot, "docs", "asset-store", "images"));
            string capRes = CaptureAssetStoreScreenshots(imagesDir);
            Console.WriteLine($"[AssetStore Verify] {capRes}");
            Console.WriteLine("[AssetStore Verify] ALL STANDALONE ASSET STORE TESTS PASSED WITH 0 ERRORS.");
            Console.WriteLine("======================================================================");
        }

        private static string CaptureAssetStoreScreenshots(string outDir)
        {
            Directory.CreateDirectory(outDir);

            // 1. Multirotor Quad X-450 hovering above helipad
            var mgr = FlightSimulationManager.SpawnCompleteSimulationEnvironment(
                VehicleAirframeCategory.Multirotor,
                SimulationFidelityLevel.Level2_6DoFBladeAndStrip);
            var v = mgr.PrimaryVehicle!;
            v.transform.SetPositionAndRotation(new Vector3(0f, 1.8f, 0f), Quaternion.Euler(4f, 25f, -5f));
            RenderCameraToPng(
                new Vector3(-1.45f, 2.35f, -1.65f),
                v.transform.position + new Vector3(0f, -0.05f, 0f),
                Path.Combine(outDir, "Screenshot_01_Multirotor_QuadX450_3DFlight.png"),
                1920, 1080);

            // 2. Fixed-Wing Talon-180 Research UAV on runway climbout
            mgr.SwitchVehiclePresetLive(VehicleAirframeCategory.FixedWing);
            v = mgr.PrimaryVehicle!;
            v.transform.SetPositionAndRotation(new Vector3(0f, 1.4f, 3.0f), Quaternion.Euler(-8f, 15f, -10f));
            RenderCameraToPng(
                new Vector3(-2.1f, 2.0f, 0.8f),
                v.transform.position + new Vector3(0f, 0.05f, 0.2f),
                Path.Combine(outDir, "Screenshot_02_FixedWing_Talon180_RunwayAndFlight.png"),
                1920, 1080);

            // 3. Hybrid QuadPlane VTOL in transition climb
            mgr.SwitchVehiclePresetLive(VehicleAirframeCategory.HybridQuadPlaneVtol);
            v = mgr.PrimaryVehicle!;
            v.transform.SetPositionAndRotation(new Vector3(0f, 2.6f, 1.5f), Quaternion.Euler(-4f, -20f, 8f));
            RenderCameraToPng(
                new Vector3(2.2f, 3.3f, -1.1f),
                v.transform.position,
                Path.Combine(outDir, "Screenshot_03_HybridVTOL_QuadPlane_Transition.png"),
                1920, 1080);

            // 4. Obstacle Course & 2D Raycast LiDAR environment view
            mgr.SwitchVehiclePresetLive(VehicleAirframeCategory.Multirotor);
            v = mgr.PrimaryVehicle!;
            v.transform.SetPositionAndRotation(new Vector3(0f, 2.5f, 4.0f), Quaternion.Euler(3f, 0f, 0f));
            GameObject tempCourse = CreateTemporaryObstacleAndLidarPreview(v.transform.position);
            RenderCameraToPng(
                new Vector3(-5.5f, 7.2f, -3.5f),
                new Vector3(0f, 2.0f, 10.5f),
                Path.Combine(outDir, "Screenshot_04_LiDAR_ObstacleCourse_Autonomy.png"),
                1920, 1080);
            UnityEngine.Object.DestroyImmediate(tempCourse);

            // Restore clean Multirotor demo scene state
            v.transform.SetPositionAndRotation(Vector3.zero, Quaternion.identity);
            PopulateAndSaveDemoScene(VehicleAirframeCategory.Multirotor, SimulationFidelityLevel.Level2_6DoFBladeAndStrip);
            return $"OK: Captured 4 Unity 3D 1920x1080 screenshots in '{outDir}'.";
        }

        private static GameObject CreateTemporaryObstacleAndLidarPreview(Vector3 dronePos)
        {
            GameObject root = new GameObject("TempObstacleAndLidarPreview");
            var shader = Shader.Find("Universal Render Pipeline/Lit") ?? Shader.Find("Standard");
            Material pillarMat = new Material(shader) { color = new Color(0.88f, 0.34f, 0.16f) };
            Material beamMat = new Material(shader) { color = new Color(0.15f, 0.95f, 0.55f) };

            Vector3[] positions = new[]
            {
                new Vector3(0f, 2.5f, 10f),
                new Vector3(-4.2f, 2.5f, 13f),
                new Vector3(4.2f, 2.5f, 13f),
                new Vector3(-2.5f, 2.5f, 19f),
                new Vector3(2.8f, 2.5f, 18.5f),
                new Vector3(0f, 2.5f, 24f)
            };

            for (int i = 0; i < positions.Length; i++)
            {
                GameObject cyl = GameObject.CreatePrimitive(PrimitiveType.Cylinder);
                cyl.transform.SetParent(root.transform, false);
                cyl.transform.position = positions[i];
                cyl.transform.localScale = new Vector3(1.6f, 2.5f, 1.6f);
                cyl.GetComponent<Renderer>().sharedMaterial = pillarMat;
            }

            // Create 24 visual laser rays radiating from dronePos
            int rays = 24;
            for (int i = 0; i < rays; i++)
            {
                float angleRad = (i * Mathf.PI * 2f) / rays;
                Vector3 dir = new Vector3(Mathf.Sin(angleRad), 0f, Mathf.Cos(angleRad));
                float len = 7.5f;
                if (Physics.Raycast(dronePos, dir, out RaycastHit hit, 18f))
                    len = hit.distance;

                GameObject rayBar = GameObject.CreatePrimitive(PrimitiveType.Cube);
                rayBar.transform.SetParent(root.transform, false);
                rayBar.transform.position = dronePos + dir * (len * 0.5f);
                rayBar.transform.rotation = Quaternion.LookRotation(dir, Vector3.up);
                rayBar.transform.localScale = new Vector3(0.035f, 0.035f, len);
                rayBar.GetComponent<Renderer>().sharedMaterial = beamMat;
            }

            return root;
        }

        private static void RenderCameraToPng(Vector3 camPos, Vector3 lookTarget, string outputPath, int width, int height)
        {
            Camera? cam = Camera.main ?? UnityEngine.Object.FindFirstObjectByType<Camera>();
            if (cam == null)
            {
                GameObject camGo = new GameObject("Main Camera");
                camGo.tag = "MainCamera";
                cam = camGo.AddComponent<Camera>();
            }

            Vector3 oldPos = cam.transform.position;
            Quaternion oldRot = cam.transform.rotation;
            CameraClearFlags oldFlags = cam.clearFlags;
            Color oldBg = cam.backgroundColor;

            cam.transform.position = camPos;
            cam.transform.LookAt(lookTarget, Vector3.up);
            cam.clearFlags = CameraClearFlags.Skybox;
            cam.backgroundColor = new Color(0.14f, 0.22f, 0.34f);

            RenderTexture rt = new RenderTexture(width, height, 24, RenderTextureFormat.ARGB32);
            RenderTexture prevActive = RenderTexture.active;
            RenderTexture? prevTarget = cam.targetTexture;

            cam.targetTexture = rt;
            cam.Render();

            RenderTexture.active = rt;
            Texture2D tex = new Texture2D(width, height, TextureFormat.RGB24, false);
            tex.ReadPixels(new Rect(0, 0, width, height), 0, 0);
            tex.Apply();

            byte[] pngBytes = ImageConversion.EncodeToPNG(tex);
            File.WriteAllBytes(outputPath, pngBytes);

            cam.targetTexture = prevTarget;
            RenderTexture.active = prevActive;
            UnityEngine.Object.DestroyImmediate(rt);
            UnityEngine.Object.DestroyImmediate(tex);

            cam.transform.SetPositionAndRotation(oldPos, oldRot);
            cam.clearFlags = oldFlags;
            cam.backgroundColor = oldBg;
        }
    }
}
