// Licensed under the Apache License, Version 2.0.
#nullable enable
using System;
using System.Collections.Concurrent;
using System.Diagnostics;
using System.IO;
using UnityEngine;
using FlightDynamics6DoF.Core.Gnc;
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
        public string WorkspaceRootPath = "/Users/byabya/UnityDrone";

        [Header("PX4-Autopilot Configuration")]
        [Tooltip("Path to local PX4-Autopilot repository root.")]
        public string Px4RepositoryPath = "/Users/byabya/UnityDrone/PX4-Autopilot";

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

        public string Px4BinaryPath =>
            Path.Combine(Px4RepositoryPath, "build", "px4_sitl_default", "bin", "px4");

        public bool IsPx4BinaryBuilt => File.Exists(Px4BinaryPath);

        private string ManagerScriptPath => Path.Combine(WorkspaceRootPath, "scripts", "px4-sitl.sh");

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
            if (TargetVehicle == null)
                TargetVehicle = FindFirstObjectByType<FlightDynamicsVehicle>();
        }

        private void Start()
        {
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
            if (TargetVehicle == null) return;
            try
            {
                string configDir = Path.Combine(WorkspaceRootPath, "docker", "config");
                Directory.CreateDirectory(configDir);
                string caScript = WrenchControlAllocator.GeneratePx4AirframeParameterScript(
                    TargetVehicle.ActiveConfig,
                    TargetVehicle.ActiveBakedDatabase);
                File.WriteAllText(Path.Combine(configDir, "unity_airframe.params"), caScript);
            }
            catch (Exception ex)
            {
                Debug.LogWarning($"[FlightDynamics6DoF] Could not write docker/config/unity_airframe.params: {ex.Message}");
            }
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
            return StartPx4SitlProcess();
        }

        private bool RunContainerAction(string actionVerb, string summaryMessage)
        {
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
        /// Completely stops active containers and shuts down the Colima VM profile (reclaiming Mac RAM and CPU).
        /// </summary>
        public void ShutdownColimaVm()
        {
            StopRos2OffboardNode();
            try
            {
                string envPrefix = GetBackendEnvPrefix();
                var psi = new ProcessStartInfo
                {
                    FileName = "/bin/bash",
                    Arguments = $"-lc \"{envPrefix}'{ManagerScriptPath}' vm-stop\"",
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
                _lastConsoleSummary = "Completely shut down Colima VM profile (flightdynamics).";
                Debug.Log($"[FlightDynamics6DoF] {_lastConsoleSummary}");
            }
            catch (Exception ex)
            {
                Debug.LogWarning($"[FlightDynamics6DoF] Failed shutting down Colima VM: {ex.Message}");
            }
        }

        /// <summary>
        /// Sends a PX4 NuttX/POSIX shell command (`pxh>`) to `unity-px4-sitl` (via Colima `nerdctl` or Apple `container`)
        /// or to the local PX4 process stdin.
        /// </summary>
        public void SendPxhCommand(string command)
        {
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
        /// Starts the ROS 2 Humble Offboard Trajectory Node (`unity_px4_offboard_node.py`) inside the `unity-ros2-dds` container.
        /// </summary>
        public void StartRos2OffboardNode(float altitudeMeters = 10.0f, float orbitRadiusMeters = 8.0f)
        {
            StopRos2OffboardNode();
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
