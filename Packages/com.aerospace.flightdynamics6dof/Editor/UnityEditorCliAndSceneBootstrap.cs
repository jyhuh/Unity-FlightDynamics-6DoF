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
        private static readonly string CmdFilePath = Path.Combine(Directory.GetCurrentDirectory(), "Temp", "fd6dof_cli_cmd.txt");
        private static readonly string ResFilePath = Path.Combine(Directory.GetCurrentDirectory(), "Temp", "fd6dof_cli_res.txt");
        private static double _lastPollTime;

        static UnityEditorCliAndSceneBootstrap()
        {
            Application.runInBackground = true;
            EditorApplication.delayCall += AutoInitializeDemoSceneIfNeeded;
            EditorApplication.update += PollCliCommandFile;
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

            // If we already saved DemoScenePath on disk, open it; otherwise populate current scene and save it
            if (File.Exists(Path.Combine(Directory.GetCurrentDirectory(), DemoScenePath)))
            {
                var activeScene = EditorSceneManager.GetActiveScene();
                if (string.IsNullOrEmpty(activeScene.path))
                {
                    EditorSceneManager.OpenScene(DemoScenePath);
                    return;
                }
            }

            PopulateAndSaveDemoScene(
                VehicleAirframeCategory.Multirotor,
                SimulationFidelityLevel.Level2_6DoFBladeAndStrip);
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

            string scenesDir = Path.Combine(Directory.GetCurrentDirectory(), "Assets", "Scenes");
            if (!Directory.Exists(scenesDir))
                Directory.CreateDirectory(scenesDir);

            var scene = EditorSceneManager.GetActiveScene();
            EditorSceneManager.MarkSceneDirty(scene);
            EditorSceneManager.SaveScene(scene, DemoScenePath);

            EditorBuildSettings.scenes = new[]
            {
                new EditorBuildSettingsScene(DemoScenePath, true)
            };

            AssetDatabase.SaveAssets();
            var db = mgr.PrimaryVehicle != null ? mgr.PrimaryVehicle.ActiveBakedDatabase : null;
            string summary = db != null
                ? $"Scene='{DemoScenePath}' | Vehicle='{db.AircraftName}' | Fidelity='{fidelity}' | StaticMargin={db.StaticMarginFraction * 100.0:F1}% MAC | CL_alpha={db.CLAlpha:F2}/rad | Cm_alpha={db.CmAlpha:F3}/rad"
                : $"Scene='{DemoScenePath}' populated.";

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
    }
}
