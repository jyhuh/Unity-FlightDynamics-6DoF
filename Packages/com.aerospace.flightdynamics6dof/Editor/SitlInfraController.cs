// Licensed under the Apache License, Version 2.0.
#nullable enable
using System;
using System.Collections.Concurrent;
using System.Collections.Generic;
using System.Diagnostics;
using System.IO;
using System.Text;
using System.Threading.Tasks;
using FlightDynamics6DoF.Unity;
using UnityEngine;
using Debug = UnityEngine.Debug;

namespace FlightDynamics6DoF.Editor
{
    public class ContainerStateInfo
    {
        public string Name = "";
        public string RawStatus = "STOPPED";
        public string Image = "";
        public string Ports = "";
        public bool IsRunning => !string.IsNullOrEmpty(RawStatus) &&
            (RawStatus.StartsWith("Up", StringComparison.OrdinalIgnoreCase) ||
             RawStatus.StartsWith("running", StringComparison.OrdinalIgnoreCase));
    }

    /// <summary>
    /// Encapsulates container/VM status polling, script execution (px4-sitl.sh),
    /// and thread-safe console log buffering independently from the EditorWindow UI.
    /// </summary>
    public sealed class SitlInfraController
    {
        public ContainerRuntimeBackend Backend = ContainerRuntimeBackend.AutoDetect;
        public bool AutoRefreshStatus = true;

        public string DetectedBackendName { get; private set; } = "colima";
        public string VmStatus { get; private set; } = "CHECKING...";
        public string ContainerdStatus { get; private set; } = "CHECKING...";
        public bool IsQueryingStatus { get; private set; }
        public bool IsOperationRunning => _isOperationRunning;
        public string LastStatusSummary => _lastStatusSummary;
        public string ConsoleText => _consoleLogBuffer.ToString();

        private readonly Dictionary<string, ContainerStateInfo> _containers =
            new Dictionary<string, ContainerStateInfo>(StringComparer.OrdinalIgnoreCase);

        private double _lastStatusQueryTime;

        private static readonly StringBuilder _consoleLogBuffer = new StringBuilder();
        private static readonly ConcurrentQueue<string> _pendingLogLines = new ConcurrentQueue<string>();
        private static readonly ConcurrentQueue<Action> _mainThreadActions = new ConcurrentQueue<Action>();
        private static string _lastStatusSummary = "Ready";
        private static bool _isOperationRunning;

        public bool IsContainerRunning(string name) =>
            _containers.TryGetValue(name, out var info) && info.IsRunning;

        public void ClearConsole() => _consoleLogBuffer.Clear();

        /// <summary>
        /// Must be called from the Unity main thread (e.g., EditorApplication.update).
        /// </summary>
        public bool Tick(double timeSinceStartup)
        {
            bool shouldRepaint = false;

            while (_mainThreadActions.TryDequeue(out var action))
            {
                action?.Invoke();
                shouldRepaint = true;
            }

            while (_pendingLogLines.TryDequeue(out string? line))
            {
                if (!string.IsNullOrEmpty(line))
                {
                    _consoleLogBuffer.AppendLine(line);
                    _lastStatusSummary = line;
                    shouldRepaint = true;
                }
            }

            if (AutoRefreshStatus && !IsQueryingStatus)
            {
                double refreshInterval = _isOperationRunning ? 2.0 : 3.5;
                if (timeSinceStartup - _lastStatusQueryTime >= refreshInterval)
                    TriggerStatusQuery(timeSinceStartup);
            }

            return shouldRepaint;
        }

        public void TriggerStatusQuery(double timeSinceStartup)
        {
            if (IsQueryingStatus) return;
            IsQueryingStatus = true;

            string scriptPath;
            string wsRoot;
            string envPrefix;
            string fallbackBackend;
            try
            {
                _lastStatusQueryTime = timeSinceStartup;
                scriptPath = GetManagerScriptPath();
                wsRoot = GetWorkspaceRoot();
                envPrefix = GetBackendEnvPrefix();
                fallbackBackend = DetectedBackendName;
            }
            catch
            {
                IsQueryingStatus = false;
                return;
            }

            Task.Run(() =>
            {
                string newBackend = fallbackBackend;
                string newVm = "STOPPED";
                string newCd = "INACTIVE";
                var newContainers = new Dictionary<string, ContainerStateInfo>(StringComparer.OrdinalIgnoreCase);

                try
                {
                    if (File.Exists(scriptPath))
                    {
                        var psi = new ProcessStartInfo
                        {
                            FileName = "/bin/bash",
                            Arguments = $"-lc \"export PATH=/opt/homebrew/bin:/usr/local/bin:$PATH; {envPrefix}'{scriptPath}' status-info\"",
                            WorkingDirectory = wsRoot,
                            UseShellExecute = false,
                            RedirectStandardOutput = true,
                            RedirectStandardError = true,
                            CreateNoWindow = true
                        };
                        using var proc = Process.Start(psi);
                        if (proc != null)
                        {
                            var stdoutLines = new List<string>();
                            var lineLock = new object();
                            proc.OutputDataReceived += (_, e) =>
                            {
                                if (e.Data != null)
                                {
                                    lock (lineLock) stdoutLines.Add(e.Data);
                                }
                            };
                            proc.BeginOutputReadLine();
                            proc.BeginErrorReadLine();

                            if (!proc.WaitForExit(10000))
                            {
                                try { proc.Kill(); } catch { }
                            }
                            else
                            {
                                proc.WaitForExit();
                            }

                            List<string> snapshot;
                            lock (lineLock) snapshot = new List<string>(stdoutLines);

                            foreach (string rawLine in snapshot)
                            {
                                string line = rawLine.Trim();
                                if (line.StartsWith("BACKEND=")) newBackend = line.Substring("BACKEND=".Length).Trim();
                                else if (line.StartsWith("VM_STATUS=")) newVm = line.Substring("VM_STATUS=".Length).Trim();
                                else if (line.StartsWith("CONTAINERD_STATUS=")) newCd = line.Substring("CONTAINERD_STATUS=".Length).Trim();
                                else if (line.Contains("|"))
                                {
                                    string[] parts = line.Split('|');
                                    if (parts.Length >= 2 && !string.IsNullOrWhiteSpace(parts[0]))
                                    {
                                        string cname = parts[0].Trim();
                                        newContainers[cname] = new ContainerStateInfo
                                        {
                                            Name = cname,
                                            RawStatus = parts[1].Trim(),
                                            Image = parts.Length > 2 ? parts[2].Trim() : "",
                                            Ports = parts.Length > 3 ? parts[3].Trim() : ""
                                        };
                                    }
                                }
                            }
                        }
                    }
                }
                catch
                {
                    newVm = "ERROR";
                    newCd = "ERROR";
                }
                finally
                {
                    _mainThreadActions.Enqueue(() =>
                    {
                        DetectedBackendName = newBackend;
                        VmStatus = newVm;
                        ContainerdStatus = newCd;
                        _containers.Clear();
                        foreach (var kv in newContainers) _containers[kv.Key] = kv.Value;
                        IsQueryingStatus = false;
                    });
                }
            });
        }

        public void RunScriptAction(string scriptArgs, string summary, Func<double> getTimeSinceStartup)
        {
            string scriptPath = GetManagerScriptPath();
            string wsRoot = GetWorkspaceRoot();
            if (!File.Exists(scriptPath))
            {
                _pendingLogLines.Enqueue($"[Error] Script not found: {scriptPath}");
                return;
            }

            _lastStatusSummary = summary;
            _pendingLogLines.Enqueue($"\n>>> {summary}");
            _isOperationRunning = true;

            try
            {
                var psi = new ProcessStartInfo
                {
                    FileName = "/bin/bash",
                    Arguments = $"-lc \"{GetBackendEnvPrefix()}'{scriptPath}' {scriptArgs}\"",
                    WorkingDirectory = wsRoot,
                    UseShellExecute = false,
                    RedirectStandardOutput = true,
                    RedirectStandardError = true,
                    CreateNoWindow = true
                };
                var proc = Process.Start(psi);
                if (proc != null)
                {
                    proc.EnableRaisingEvents = true;
                    proc.Exited += (_, _) =>
                    {
                        _mainThreadActions.Enqueue(() =>
                        {
                            _isOperationRunning = false;
                            TriggerStatusQuery(getTimeSinceStartup());
                        });
                    };
                    proc.OutputDataReceived += (_, e) => { if (!string.IsNullOrEmpty(e.Data)) _pendingLogLines.Enqueue(e.Data); };
                    proc.ErrorDataReceived += (_, e) => { if (!string.IsNullOrEmpty(e.Data)) _pendingLogLines.Enqueue("[ERR] " + e.Data); };
                    proc.BeginOutputReadLine();
                    proc.BeginErrorReadLine();
                }
                else
                {
                    _isOperationRunning = false;
                }
            }
            catch (Exception ex)
            {
                _isOperationRunning = false;
                _pendingLogLines.Enqueue($"[Exception] {ex.Message}");
            }
        }

        public void OpenLogsInTerminal()
        {
            try
            {
                string script = $"tell application \"Terminal\" to do script \"cd '{GetWorkspaceRoot()}' && ./scripts/px4-sitl.sh logs\"";
                Process.Start("osascript", $"-e \"{script}\"");
            }
            catch (Exception ex)
            {
                Debug.LogWarning($"[FlightDynamics6DoF] Could not open terminal: {ex.Message}");
            }
        }

        public static string GetWorkspaceRoot()
        {
            var launcher = UnityEngine.Object.FindFirstObjectByType<Px4SitlProcessLauncher>();
            if (launcher != null && !string.IsNullOrEmpty(launcher.EffectiveWorkspaceRoot))
                return launcher.EffectiveWorkspaceRoot;
            try
            {
                string candidate = Path.GetFullPath(Path.Combine(Application.dataPath, ".."));
                if (Directory.Exists(candidate)) return candidate;
            }
            catch { }
            return Directory.GetCurrentDirectory();
        }

        private static string GetManagerScriptPath() => Path.Combine(GetWorkspaceRoot(), "scripts", "px4-sitl.sh");

        private string GetBackendEnvPrefix() => Backend switch
        {
            ContainerRuntimeBackend.ColimaContainerd => "CONTAINER_BACKEND=colima ",
            ContainerRuntimeBackend.AppleContainer => "CONTAINER_BACKEND=apple ",
            _ => "CONTAINER_BACKEND=auto "
        };
    }
}

