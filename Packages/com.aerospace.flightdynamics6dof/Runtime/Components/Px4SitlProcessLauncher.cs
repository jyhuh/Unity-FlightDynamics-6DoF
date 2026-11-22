// Licensed under the Apache License, Version 2.0.
#nullable enable
using System;
using System.Collections.Concurrent;
using System.Diagnostics;
using System.IO;
using UnityEngine;
using FlightDynamics6DoF.Core.Aerodynamics;
using FlightDynamics6DoF.Core.Gnc;
using FlightDynamics6DoF.Core.Specs;
using Debug = UnityEngine.Debug;

namespace FlightDynamics6DoF.Unity
{
    /// <summary>
    /// Selects the non-Docker container runtime (Colima containerd/nerdctl or Apple native `container`)
    /// or native binary used to run the independent PX4 SITL (`unity-px4-sitl`) and ROS 2 (`unity-ros2-dds`) stack.
    /// </summary>
    public enum ContainerRuntimeBackend
    {
        AutoDetect = 0,
        ColimaContainerd = 1,
        AppleContainer = 2,
        NativeBinary = 3
    }

    /// <summary>
    /// Manages the external PX4-Autopilot SITL (`unity-px4-sitl`) and ROS 2 Humble (`unity-ros2-dds`)
    /// containers via Colima (`containerd`/`nerdctl`) or Apple's native `container` CLI (zero Docker dependency).
    /// Unity and FlightDynamics6DoF.Core.dll act strictly as the 6-DOF Physics, Aerodynamics &amp; Virtual Sensor Environment
    /// (streaming HIL_SENSOR / HIL_GPS over TCP 4560), while the `unity-px4-sitl` container runs EKF2, Flight Control,
    /// and Control Allocation (returning HIL_ACTUATOR_CONTROLS) and bridges uORB topics to `unity-ros2-dds` (UDP 8888).
    /// </summary>
    [DisallowMultipleComponent]
    public class Px4SitlProcessLauncher : MonoBehaviour
    {
        [Header("Non-Docker Container Runtime (Colima containerd or Apple 'container')")]
        [Tooltip("Selects Colima (containerd + nerdctl) or Apple's native 'container' CLI (Virtualization.framework).")]
        public ContainerRuntimeBackend RuntimeBackend = ContainerRuntimeBackend.AutoDetect;

        [Tooltip("Workspace root containing scripts/px4-sitl.sh and PX4-Autopilot.")]
        public string WorkspaceRootPath = "";

        [Header("PX4-Autopilot Configuration")]
        [Tooltip("Path to local PX4-Autopilot repository root.")]
        public string Px4RepositoryPath = "";

        [Tooltip("Automatically launch the PX4 SITL container/process when entering Play Mode.")]
        public bool AutoLaunchPx4OnPlay = true;

        [Tooltip("PX4 SITL airframe model (e.g., iris, standard_vtol, plane).")]
        public string Px4AirframeModel = "iris";

        [Header("Target Environment Vehicle")]
        public FlightDynamicsVehicle? TargetVehicle;

        private Process? _px4Process;
        private Process? _ros2OffboardProcess;
        private StreamWriter? _px4Stdin;
        private bool _containerStackMarkedRunning;
        private readonly ConcurrentQueue<string> _consoleLines = new ConcurrentQueue<string>();
        private string _lastConsoleSummary = "Ready — run './scripts/px4-sitl.sh up' (Colima / Apple container)";

        public bool IsPx4ProcessRunning =>
            RuntimeBackend != ContainerRuntimeBackend.NativeBinary
                ? _containerStackMarkedRunning
                : (_px4Process != null && !_px4Process.HasExited);

        public string LastConsoleSummary => _lastConsoleSummary;

        public string EffectiveWorkspaceRoot
        {
            get
            {
                try
                {
                    string projectRoot = Path.GetFullPath(Path.Combine(Application.dataPath, ".."));
                    if (Directory.Exists(projectRoot))
                    {
                        // Only honor an explicit WorkspaceRootPath override if it actually contains scripts/px4-sitl.sh
                        // AND equals the current Unity project root (prevents stale serialized paths from another project).
                        if (!string.IsNullOrEmpty(WorkspaceRootPath) &&
                            Directory.Exists(WorkspaceRootPath) &&
                            string.Equals(Path.GetFullPath(WorkspaceRootPath), projectRoot, StringComparison.OrdinalIgnoreCase))
                        {
                            return WorkspaceRootPath;
                        }
                        return projectRoot;
                    }
                }
                catch { }

                if (!string.IsNullOrEmpty(WorkspaceRootPath) && Directory.Exists(WorkspaceRootPath))
                    return WorkspaceRootPath;

                return WorkspaceRootPath ?? "";
            }
        }

        public string EffectivePx4RepositoryPath
        {
            get
            {
                if (!string.IsNullOrEmpty(Px4RepositoryPath) && Directory.Exists(Px4RepositoryPath))
                    return Px4RepositoryPath;

                return Path.Combine(EffectiveWorkspaceRoot, "PX4-Autopilot");
            }
        }

        public string Px4BinaryPath =>
            Path.Combine(EffectivePx4RepositoryPath, "build", "px4_sitl_default", "bin", "px4");

        public bool IsPx4BinaryBuilt => File.Exists(Px4BinaryPath);

        public bool IsSitlWorkspaceAvailable =>
            !string.IsNullOrEmpty(EffectiveWorkspaceRoot) && File.Exists(ManagerScriptPath);

        private string ManagerScriptPath => Path.Combine(EffectiveWorkspaceRoot, "scripts", "px4-sitl.sh");

        private string GetBackendEnvPrefix()
        {
            return RuntimeBackend switch
            {
                ContainerRuntimeBackend.ColimaContainerd => "CONTAINER_BACKEND=colima ",
                ContainerRuntimeBackend.AppleContainer => "CONTAINER_BACKEND=apple ",
                _ => "CONTAINER_BACKEND=auto "
            };
        }

        private void Awake()
        {
            if (string.IsNullOrEmpty(WorkspaceRootPath) || !Directory.Exists(WorkspaceRootPath))
                WorkspaceRootPath = EffectiveWorkspaceRoot;
            if (string.IsNullOrEmpty(Px4RepositoryPath) || !Directory.Exists(Px4RepositoryPath))
                Px4RepositoryPath = EffectivePx4RepositoryPath;

            if (TargetVehicle == null)
                TargetVehicle = FindFirstObjectByType<FlightDynamicsVehicle>();
        }

        private void Start()
        {
            if (!IsSitlWorkspaceAvailable && !IsPx4BinaryBuilt)
            {
                _lastConsoleSummary = "Standalone Mode Active (Built-in C# SO(3)/TECS Autopilot — zero external container dependencies)";
                if (TargetVehicle != null)
                    TargetVehicle.UseInternalFlightController = true;
                return;
            }

            ExportStage1AirframeParamsForContainer();
            if (AutoLaunchPx4OnPlay)
            {
                EnsurePx4SitlRunning();
            }
        }

        private void OnDisable()
        {
            StopRos2OffboardNode();
            if (RuntimeBackend == ContainerRuntimeBackend.NativeBinary)
                StopPx4SitlProcess();
        }

        private void OnDestroy()
        {
            StopRos2OffboardNode();
            if (RuntimeBackend == ContainerRuntimeBackend.NativeBinary)
                StopPx4SitlProcess();
        }

        private void Update()
        {
            while (_consoleLines.TryDequeue(out string? line))
            {
                if (!string.IsNullOrWhiteSpace(line))
                    _lastConsoleSummary = line;
            }
        }

        public void ExportStage1AirframeParamsForContainer()
        {
            try
            {
                string wsRoot = EffectiveWorkspaceRoot;
                if (string.IsNullOrEmpty(wsRoot)) return;
                if (!IsSitlWorkspaceAvailable && !Directory.Exists(Path.Combine(wsRoot, "docker")))
                    return;

                string configDir = Path.Combine(wsRoot, "docker", "config");
                Directory.CreateDirectory(configDir);

                if (TargetVehicle != null)
                {
                    TargetVehicle.SyncAirframeControlParameterTable(resetIfCategoryChanged: false);
                    string caScript = TargetVehicle.ControlParameterTable.GeneratePx4ParameterScript(
                        TargetVehicle.ActiveConfig,
                        TargetVehicle.ActiveBakedDatabase);

                    string activeModel = TargetVehicle.ControlParameterTable.Px4SitlModelName;
                    if (!string.IsNullOrEmpty(activeModel))
                    {
                        Px4AirframeModel = activeModel;
                        File.WriteAllText(Path.Combine(configDir, $"unity_airframe_{activeModel}.params"), caScript);
                    }
                    File.WriteAllText(Path.Combine(configDir, "unity_airframe.params"), caScript);
                }

                // Ensure per-airframe default parameter files exist so CLI container switches always stay isolated
                EnsureCanonicalAirframeParamsIfMissing(configDir, "iris", VehicleAirframeCategory.Multirotor);
                EnsureCanonicalAirframeParamsIfMissing(configDir, "plane", VehicleAirframeCategory.FixedWing);
                EnsureCanonicalAirframeParamsIfMissing(configDir, "standard_vtol", VehicleAirframeCategory.HybridQuadPlaneVtol);
            }
            catch (Exception ex)
            {
                Debug.LogWarning($"[FlightDynamics6DoF] Could not write docker/config airframe params: {ex.Message}");
            }
        }

        private static void EnsureCanonicalAirframeParamsIfMissing(string configDir, string modelName, VehicleAirframeCategory cat)
        {
            string path = Path.Combine(configDir, $"unity_airframe_{modelName}.params");
            if (File.Exists(path)) return;

            AircraftConfigSpec spec = cat switch
            {
                VehicleAirframeCategory.FixedWing => AircraftConfigSpec.CreateFixedWingUav(),
                VehicleAirframeCategory.HybridQuadPlaneVtol => AircraftConfigSpec.CreateQuadPlaneVtolMillie(),
                _ => AircraftConfigSpec.CreateStandardQuadcopterX()
            };
            var db = StabilityAndTrimAnalyzer.AnalyzeAndBake(spec);
            var table = AirframeControlParameterTable.CreateForCategory(cat, spec, db);
            File.WriteAllText(path, table.GeneratePx4ParameterScript(spec, db));
        }

        /// <summary>
        /// Ensures the 2-container stack (`unity-px4-sitl` + `unity-ros2-dds`) is running without restarting
        /// already-running containers (preserving live QGroundControl and ROS 2 connections when entering Play Mode).
        /// </summary>
        public bool EnsurePx4SitlRunning()
        {
            ExportStage1AirframeParamsForContainer();

            if (RuntimeBackend != ContainerRuntimeBackend.NativeBinary)
            {
                return RunContainerAction("ensure", $"[{RuntimeBackend}] Ensuring unity-px4-sitl (TCP 4560) + unity-ros2-dds (UDP 8888) are active...");
            }

            return StartNativePx4Process();
        }

        /// <summary>
        /// Starts or restarts either the 2-container stack (`unity-px4-sitl` + `unity-ros2-dds`) via Colima (`containerd`)
        /// or Apple's `container` CLI, or the native PX4 SITL binary.
        /// </summary>
        public bool StartPx4SitlProcess()
        {
            ExportStage1AirframeParamsForContainer();

            if (RuntimeBackend != ContainerRuntimeBackend.NativeBinary)
            {
                return RunContainerAction("up", $"[{RuntimeBackend}] Starting unity-px4-sitl ({Px4AirframeModel} -> TCP 4560) + unity-ros2-dds (UDP 8888)...");
            }

            return StartNativePx4Process();
        }

        /// <summary>
        /// Switches the PX4 SITL airframe model (e.g., iris, plane, standard_vtol) and restarts the container stack.
        /// </summary>
        public bool SwitchAirframeModelAndRestart(string modelName)
        {
            Px4AirframeModel = modelName;
            return StartPx4Only(modelName);
        }

        public bool StartPx4Only(string modelName)
        {
            Px4AirframeModel = modelName;
            ExportStage1AirframeParamsForContainer();
            if (RuntimeBackend != ContainerRuntimeBackend.NativeBinary)
                return RunContainerAction("start-px4", $"[{RuntimeBackend}] Starting/restarting unity-px4-sitl ({modelName}, isolated rootfs)...");
            return StartNativePx4Process();
        }

        public bool StopPx4Only()
        {
            if (RuntimeBackend != ContainerRuntimeBackend.NativeBinary)
                return RunContainerAction("stop-px4", $"[{RuntimeBackend}] Stopping unity-px4-sitl container...");
            return true;
        }

        public bool StartRos2Only()
        {
            if (RuntimeBackend != ContainerRuntimeBackend.NativeBinary)
                return RunContainerAction("start-ros2", $"[{RuntimeBackend}] Starting/restarting unity-ros2-dds container...");
            return false;
        }

        public bool StopRos2Only()
        {
            if (RuntimeBackend != ContainerRuntimeBackend.NativeBinary)
                return RunContainerAction("stop-ros2", $"[{RuntimeBackend}] Stopping unity-ros2-dds container...");
            return false;
        }

        public bool ResetIsolatedRootfs()
        {
            if (RuntimeBackend != ContainerRuntimeBackend.NativeBinary)
                return RunContainerAction("reset-rootfs", $"[{RuntimeBackend}] Clearing isolated PX4 rootfs caches...");
            return false;
        }

        private bool RunContainerAction(string actionVerb, string summaryMessage)
        {
            if (!IsSitlWorkspaceAvailable)
            {
                _lastConsoleSummary = "Standalone C# Autopilot Mode (external scripts/px4-sitl.sh not present in this Unity project).";
                if (TargetVehicle != null)
                    TargetVehicle.UseInternalFlightController = true;
                return false;
            }

            try
            {
                string envPrefix = GetBackendEnvPrefix();
                var psi = new ProcessStartInfo
                {
                    FileName = "/bin/bash",
                    Arguments = $"-lc \"{envPrefix}'{ManagerScriptPath}' {actionVerb} {Px4AirframeModel}\"",
                    WorkingDirectory = WorkspaceRootPath,
                    UseShellExecute = false,
                    RedirectStandardOutput = true,
                    RedirectStandardError = true,
                    CreateNoWindow = true
                };

                var proc = Process.Start(psi);
                if (proc != null)
                {
                    proc.EnableRaisingEvents = true;
                    proc.OutputDataReceived += (_, e) => { if (!string.IsNullOrEmpty(e.Data)) _consoleLines.Enqueue(e.Data); };
                    proc.ErrorDataReceived += (_, e) => { if (!string.IsNullOrEmpty(e.Data)) _consoleLines.Enqueue(e.Data); };
                    proc.BeginOutputReadLine();
                    proc.BeginErrorReadLine();
                }

                _containerStackMarkedRunning = true;
                _lastConsoleSummary = summaryMessage;
                Debug.Log($"[FlightDynamics6DoF] {_lastConsoleSummary}");
                return true;
            }
            catch (Exception ex)
            {
                _lastConsoleSummary = $"Container launch failed: {ex.Message}";
                Debug.LogError($"[FlightDynamics6DoF] {_lastConsoleSummary}");
                return false;
            }
        }

        private bool StartNativePx4Process()
        {
            StopPx4SitlProcess();

            if (!IsPx4BinaryBuilt)
            {
                _lastConsoleSummary = "Native binary not found. Use Colima/Apple Container mode or run: ./scripts/px4-sitl.sh setup";
                Debug.LogWarning($"[FlightDynamics6DoF] {_lastConsoleSummary}");
                return false;
            }

            try
            {
                string buildDir = Path.Combine(Px4RepositoryPath, "build", "px4_sitl_default");
                string rootFsDir = Path.Combine(buildDir, "tmp", "rootfs");
                Directory.CreateDirectory(rootFsDir);

                if (TargetVehicle != null)
                {
                    string caScript = WrenchControlAllocator.GeneratePx4AirframeParameterScript(
                        TargetVehicle.ActiveConfig,
                        TargetVehicle.ActiveBakedDatabase);
                    File.WriteAllText(Path.Combine(rootFsDir, "unity_airframe.params"), caScript);
                }

                string etcDir = Path.Combine(buildDir, "etc");
                var psi = new ProcessStartInfo
                {
                    FileName = Px4BinaryPath,
                    Arguments = $"-d \"{etcDir}\" -w \"{rootFsDir}\" -s etc/init.d-posix/rcS",
                    WorkingDirectory = rootFsDir,
                    UseShellExecute = false,
                    RedirectStandardInput = true,
                    RedirectStandardOutput = true,
                    RedirectStandardError = true,
                    CreateNoWindow = true
                };

                psi.EnvironmentVariables["PX4_SIM_MODEL"] = Px4AirframeModel;
                psi.EnvironmentVariables["PX4_SIMULATOR"] = "none";

                if (TargetVehicle != null && TargetVehicle.Engine != null)
                {
                    psi.EnvironmentVariables["PX4_HOME_LAT"] = TargetVehicle.Engine.OriginLatitudeDeg.ToString("F7", System.Globalization.CultureInfo.InvariantCulture);
                    psi.EnvironmentVariables["PX4_HOME_LON"] = TargetVehicle.Engine.OriginLongitudeDeg.ToString("F7", System.Globalization.CultureInfo.InvariantCulture);
                    psi.EnvironmentVariables["PX4_HOME_ALT"] = TargetVehicle.Engine.OriginAltitudeMsl.ToString("F2", System.Globalization.CultureInfo.InvariantCulture);
                }

                _px4Process = new Process { StartInfo = psi, EnableRaisingEvents = true };
                _px4Process.OutputDataReceived += (_, e) =>
                {
                    if (!string.IsNullOrEmpty(e.Data))
                        _consoleLines.Enqueue(e.Data);
                };
                _px4Process.ErrorDataReceived += (_, e) =>
                {
                    if (!string.IsNullOrEmpty(e.Data))
                        _consoleLines.Enqueue("[ERR] " + e.Data);
                };

                _px4Process.Start();
                _px4Process.BeginOutputReadLine();
                _px4Process.BeginErrorReadLine();
                _px4Stdin = _px4Process.StandardInput;

                _lastConsoleSummary = $"Native PX4 SITL started (PID {_px4Process.Id}), connecting to Unity TCP 4560...";
                Debug.Log($"[FlightDynamics6DoF] {_lastConsoleSummary}");
                return true;
            }
            catch (Exception ex)
            {
                _lastConsoleSummary = $"Failed to launch PX4 SITL: {ex.Message}";
                Debug.LogError($"[FlightDynamics6DoF] {_lastConsoleSummary}");
                return false;
            }
        }

        public void StopPx4SitlProcess()
        {
            StopRos2OffboardNode();

            if (RuntimeBackend != ContainerRuntimeBackend.NativeBinary)
            {
                if (!IsSitlWorkspaceAvailable)
                {
                    _containerStackMarkedRunning = false;
                    return;
                }
                try
                {
                    string envPrefix = GetBackendEnvPrefix();
                    var psi = new ProcessStartInfo
                    {
                        FileName = "/bin/bash",
                        Arguments = $"-lc \"{envPrefix}'{ManagerScriptPath}' down\"",
                        WorkingDirectory = WorkspaceRootPath,
                        UseShellExecute = false,
                        RedirectStandardOutput = true,
                        RedirectStandardError = true,
                        CreateNoWindow = true
                    };
                    var proc = Process.Start(psi);
                    if (proc != null)
                    {
                        proc.OutputDataReceived += (_, e) => { if (!string.IsNullOrEmpty(e.Data)) _consoleLines.Enqueue(e.Data); };
                        proc.ErrorDataReceived += (_, e) => { if (!string.IsNullOrEmpty(e.Data)) _consoleLines.Enqueue(e.Data); };
                        proc.BeginOutputReadLine();
                        proc.BeginErrorReadLine();
                    }
                    _containerStackMarkedRunning = false;
                    _lastConsoleSummary = "Stopped unity-px4-sitl and unity-ros2-dds containers.";
                }
                catch (Exception ex)
                {
                    Debug.LogWarning($"[FlightDynamics6DoF] Failed stopping container stack: {ex.Message}");
                }
                return;
            }

            if (_px4Process == null) return;
            try
            {
                if (!_px4Process.HasExited)
                {
                    _px4Stdin?.WriteLine("shutdown");
                    _px4Stdin?.Flush();
                    if (!_px4Process.WaitForExit(300))
                        _px4Process.Kill();
                }
            }
            catch
            {
                // Ignore process cleanup exceptions
            }
            finally
            {
                _px4Process.Dispose();
                _px4Process = null;
                _px4Stdin = null;
            }
        }

        /// <summary>
        /// Boots and ensures the Colima VM (flightdynamics profile) and containerd runtime are ready.
        /// </summary>
        public void StartColimaVm()
        {
            RunVmAction("vm-start", "Starting / verifying Colima VM and containerd runtime...");
        }

        /// <summary>
        /// Checks the health status of Colima VM, systemd containerd.service, and nerdctl containers.
        /// Output is piped to Unity console log and HUD summary.
        /// </summary>
        public void CheckVmStatus()
        {
            RunVmAction("vm-status", "Checking Colima VM & containerd service status...");
        }

        /// <summary>
        /// Force reloads systemd, restarts containerd, and re-enables containerd.service inside the Colima VM.
        /// Useful when containerd enters an inactive/dead state.
        /// </summary>
        public void RepairContainerdService()
        {
            RunVmAction("vm-repair", "Repairing and restarting containerd daemon inside Colima VM...");
        }

        /// <summary>
        /// Completely stops active containers and shuts down the Colima VM profile (reclaiming Mac RAM and CPU).
        /// </summary>
        public void ShutdownColimaVm()
        {
            StopRos2OffboardNode();
            _containerStackMarkedRunning = false;
            RunVmAction("vm-stop", "Completely shut down Colima VM profile (flightdynamics).");
        }

        private bool RunVmAction(string actionVerb, string summaryMessage)
        {
            if (!IsSitlWorkspaceAvailable)
            {
                _lastConsoleSummary = "Standalone C# Autopilot Mode (external Colima VM script not present in this Unity project).";
                return false;
            }

            try
            {
                string envPrefix = GetBackendEnvPrefix();
                var psi = new ProcessStartInfo
                {
                    FileName = "/bin/bash",
                    Arguments = $"-lc \"{envPrefix}'{ManagerScriptPath}' {actionVerb}\"",
                    WorkingDirectory = WorkspaceRootPath,
                    UseShellExecute = false,
                    RedirectStandardOutput = true,
                    RedirectStandardError = true,
                    CreateNoWindow = true
                };
                var proc = Process.Start(psi);
                if (proc != null)
                {
                    proc.EnableRaisingEvents = true;
                    proc.OutputDataReceived += (_, e) => { if (!string.IsNullOrEmpty(e.Data)) _consoleLines.Enqueue(e.Data); };
                    proc.ErrorDataReceived += (_, e) => { if (!string.IsNullOrEmpty(e.Data)) _consoleLines.Enqueue(e.Data); };
                    proc.BeginOutputReadLine();
                    proc.BeginErrorReadLine();
                }
                _lastConsoleSummary = summaryMessage;
                Debug.Log($"[FlightDynamics6DoF] {_lastConsoleSummary}");
                return true;
            }
            catch (Exception ex)
            {
                _lastConsoleSummary = $"Action '{actionVerb}' failed: {ex.Message}";
                Debug.LogWarning($"[FlightDynamics6DoF] {_lastConsoleSummary}");
                return false;
            }
        }

        private bool TryExecuteInternalFlightControllerCommand(string command)
        {
            if (TargetVehicle == null || TargetVehicle.Engine == null || TargetVehicle.FlightController == null)
                return false;

            var mavNode = TargetVehicle.GetComponent<MavlinkNetworkNode>();
            bool externalPx4ActivelyControlling = mavNode != null && mavNode.IsConnected && mavNode.Bridge.TotalHilActuatorFramesReceived > 0;
            if (externalPx4ActivelyControlling && !TargetVehicle.UseInternalFlightController)
                return false;

            TargetVehicle.UseInternalFlightController = true;
            var fc = TargetVehicle.FlightController;
            var st = TargetVehicle.Engine.State;
            double yaw = st.AttitudeBodyToNed.ToEulerZyx().Z;
            string cmd = command.Trim().ToLowerInvariant();

            if (cmd == "commander arm")
            {
                fc.Arm(st.PositionNed, yaw);
                _lastConsoleSummary = "[Standalone C# Autopilot] Armed";
                return true;
            }
            if (cmd == "commander disarm")
            {
                fc.Disarm();
                _lastConsoleSummary = "[Standalone C# Autopilot] Disarmed";
                return true;
            }
            if (cmd.StartsWith("commander takeoff"))
            {
                fc.CommandTakeoff(st.PositionNed, yaw, TargetVehicle.InitialTakeoffAltitudeMeters);
                _lastConsoleSummary = $"[Standalone C# Autopilot] Takeoff -> {TargetVehicle.InitialTakeoffAltitudeMeters:F1}m AGL";
                return true;
            }
            if (cmd.Contains("auto:loiter"))
            {
                fc.CommandLoiterHold(st.PositionNed, yaw);
                _lastConsoleSummary = "[Standalone C# Autopilot] Position Hold / Loiter";
                return true;
            }
            if (cmd.Contains("auto:rtl"))
            {
                fc.CommandReturnToLaunch();
                _lastConsoleSummary = "[Standalone C# Autopilot] Return to Launch (RTL)";
                return true;
            }
            if (cmd == "commander land")
            {
                fc.CommandAutoLand(st.PositionNed);
                _lastConsoleSummary = "[Standalone C# Autopilot] Auto-Land";
                return true;
            }
            if (cmd.Contains("vtol_transition fixedwing"))
            {
                fc.CommandVtolTransition(true, st.PositionNed, yaw);
                _lastConsoleSummary = "[Standalone C# Autopilot] VTOL Transition -> Fixed-Wing Cruise";
                return true;
            }
            if (cmd.Contains("vtol_transition multicopter"))
            {
                fc.CommandVtolTransition(false, st.PositionNed, yaw);
                _lastConsoleSummary = "[Standalone C# Autopilot] VTOL Transition -> Multicopter Hover";
                return true;
            }
            if (cmd == "commander check" || cmd == "mavlink status")
            {
                _lastConsoleSummary = $"[Standalone C# Autopilot] Mode={fc.CurrentMode}, Armed={fc.IsArmed}, Alt={st.AltitudeAglMeters:F1}m, V={st.AirspeedMps:F1}m/s";
                return true;
            }
            return false;
        }

        /// <summary>
        /// Sends a PX4 NuttX/POSIX shell command (`pxh>`) to `unity-px4-sitl` (via Colima `nerdctl` or Apple `container`),
        /// or executes it directly on the built-in C# Flight Controller when running in Standalone mode.
        /// </summary>
        public void SendPxhCommand(string command)
        {
            bool handledInternally = TryExecuteInternalFlightControllerCommand(command);
            if (!IsSitlWorkspaceAvailable && RuntimeBackend != ContainerRuntimeBackend.NativeBinary)
                return;
            if (handledInternally && TargetVehicle != null && TargetVehicle.UseInternalFlightController)
            {
                var mavNode = TargetVehicle.GetComponent<MavlinkNetworkNode>();
                if (mavNode == null || !mavNode.IsConnected)
                    return;
            }

            if (RuntimeBackend != ContainerRuntimeBackend.NativeBinary)
            {
                try
                {
                    string envPrefix = GetBackendEnvPrefix();
                    var psi = new ProcessStartInfo
                    {
                        FileName = "/bin/bash",
                        Arguments = $"-lc \"{envPrefix}'{ManagerScriptPath}' pxh {command}\"",
                        WorkingDirectory = WorkspaceRootPath,
                        UseShellExecute = false,
                        RedirectStandardOutput = true,
                        RedirectStandardError = true,
                        CreateNoWindow = true
                    };
                    var proc = Process.Start(psi);
                    if (proc != null)
                    {
                        proc.OutputDataReceived += (_, e) => { if (!string.IsNullOrEmpty(e.Data)) _consoleLines.Enqueue(e.Data); };
                        proc.ErrorDataReceived += (_, e) => { if (!string.IsNullOrEmpty(e.Data)) _consoleLines.Enqueue(e.Data); };
                        proc.BeginOutputReadLine();
                        proc.BeginErrorReadLine();
                    }
                    _lastConsoleSummary = $"[unity-px4-sitl pxh>] {command}";
                }
                catch (Exception ex)
                {
                    Debug.LogWarning($"[FlightDynamics6DoF] Failed sending container pxh command: {ex.Message}");
                }
                return;
            }

            if (!IsPx4ProcessRunning || _px4Stdin == null)
            {
                Debug.LogWarning($"[FlightDynamics6DoF] Cannot send pxh command '{command}': PX4 SITL process is not running.");
                return;
            }

            try
            {
                _px4Stdin.WriteLine(command);
                _px4Stdin.Flush();
                _lastConsoleSummary = $"pxh> {command}";
            }
            catch (Exception ex)
            {
                Debug.LogWarning($"[FlightDynamics6DoF] Failed sending pxh command: {ex.Message}");
            }
        }

        /// <summary>
        /// Starts the ROS 2 Humble Offboard Trajectory Node (`unity_px4_offboard_node.py`) inside the `unity-ros2-dds` container,
        /// or executes a built-in C# circular orbit mission when running in Standalone mode.
        /// </summary>
        public void StartRos2OffboardNode(float altitudeMeters = 10.0f, float orbitRadiusMeters = 8.0f)
        {
            StopRos2OffboardNode();

            var mavNode = TargetVehicle != null ? TargetVehicle.GetComponent<MavlinkNetworkNode>() : null;
            bool externalPx4Connected = mavNode != null && mavNode.IsConnected && mavNode.Bridge.TotalHilActuatorFramesReceived > 0;
            if ((!IsSitlWorkspaceAvailable || !externalPx4Connected) && TargetVehicle != null && TargetVehicle.Engine != null && TargetVehicle.FlightController != null)
            {
                TargetVehicle.UseInternalFlightController = true;
                TargetVehicle.FlightController.CommandCircularOrbit(
                    TargetVehicle.Engine.State.PositionNed,
                    altitudeMeters,
                    orbitRadiusMeters,
                    16);
                _lastConsoleSummary = $"[Standalone C# Autopilot] Started Circular Orbit Mission (Alt={altitudeMeters:F1}m, R={orbitRadiusMeters:F1}m)";
                Debug.Log($"[FlightDynamics6DoF] {_lastConsoleSummary}");
                if (!IsSitlWorkspaceAvailable)
                    return;
            }

            if (!IsSitlWorkspaceAvailable)
                return;

            try
            {
                string envPrefix = GetBackendEnvPrefix();
                var psi = new ProcessStartInfo
                {
                    FileName = "/bin/bash",
                    Arguments = $"-lc \"{envPrefix}'{ManagerScriptPath}' ros2-offboard --altitude {altitudeMeters:F1} --radius {orbitRadiusMeters:F1}\"",
                    WorkingDirectory = WorkspaceRootPath,
                    UseShellExecute = false,
                    RedirectStandardOutput = true,
                    RedirectStandardError = true,
                    CreateNoWindow = true
                };

                _ros2OffboardProcess = new Process { StartInfo = psi, EnableRaisingEvents = true };
                _ros2OffboardProcess.OutputDataReceived += (_, e) =>
                {
                    if (!string.IsNullOrEmpty(e.Data))
                        _consoleLines.Enqueue("[ROS2] " + e.Data);
                };
                _ros2OffboardProcess.ErrorDataReceived += (_, e) =>
                {
                    if (!string.IsNullOrEmpty(e.Data))
                        _consoleLines.Enqueue("[ROS2] " + e.Data);
                };
                _ros2OffboardProcess.Start();
                _ros2OffboardProcess.BeginOutputReadLine();
                _ros2OffboardProcess.BeginErrorReadLine();

                _lastConsoleSummary = $"[unity-ros2-dds] Started ROS 2 Offboard Node (Alt={altitudeMeters:F1}m, R={orbitRadiusMeters:F1}m)";
                Debug.Log($"[FlightDynamics6DoF] {_lastConsoleSummary}");
            }
            catch (Exception ex)
            {
                _lastConsoleSummary = $"Failed starting ROS 2 Offboard node: {ex.Message}";
                Debug.LogWarning($"[FlightDynamics6DoF] {_lastConsoleSummary}");
            }
        }

        public void StopRos2OffboardNode()
        {
            if (TargetVehicle != null && TargetVehicle.UseInternalFlightController && TargetVehicle.Engine != null && TargetVehicle.FlightController != null &&
                TargetVehicle.FlightController.CurrentMode == AutopilotFlightMode.WaypointMission)
            {
                var st = TargetVehicle.Engine.State;
                TargetVehicle.FlightController.CommandLoiterHold(st.PositionNed, st.AttitudeBodyToNed.ToEulerZyx().Z);
                _lastConsoleSummary = "[Standalone C# Autopilot] Stopped Orbit Mission -> Position Hold";
            }

            if (_ros2OffboardProcess == null) return;
            try
            {
                if (!_ros2OffboardProcess.HasExited)
                    _ros2OffboardProcess.Kill();
            }
            catch
            {
                // Ignore
            }
            finally
            {
                _ros2OffboardProcess.Dispose();
                _ros2OffboardProcess = null;
            }
        }
    }
}
