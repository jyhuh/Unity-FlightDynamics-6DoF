// Licensed under the Apache License, Version 2.0.
#nullable enable
using System;
using System.Collections.Concurrent;
using System.Diagnostics;
using System.IO;
using System.Net;
using System.Net.Sockets;
using System.Threading;
using UnityEngine;
using FlightDynamics6DoF.Core.Dynamics;
using FlightDynamics6DoF.Core.Mavlink;
using FlightDynamics6DoF.Core.Math;
using Debug = UnityEngine.Debug;

namespace FlightDynamics6DoF.Unity
{
    public enum NetworkTransportType
    {
        TcpPx4SitlServer = 0,
        TcpPx4SitlClient = 1,
        UdpGcsOrSitl = 2
    }

    /// <summary>
    /// Unity MonoBehaviour providing MAVLink v2 networking for:
    /// 1) External PX4 SITL HIL Lockstep/Async connection over TCP 4560 (Default — PX4 controls the drone, Unity provides the 6-DOF environment),
    /// 2) External ArduPilot SITL connection,
    /// 3) Optional Direct QGroundControl / Mission Planner connection (UDP 14550).
    /// </summary>
    [DisallowMultipleComponent]
    public class MavlinkNetworkNode : MonoBehaviour
    {
        [Header("Target Environment Vehicle")]
        public FlightDynamicsVehicle? Vehicle;

        [Header("MAVLink Bridge & Transport Configuration (Default: PX4 SITL TCP 4560)")]
        public MavlinkBridgeMode BridgeMode = MavlinkBridgeMode.ExternalSitlHil;
        public NetworkTransportType Transport = NetworkTransportType.TcpPx4SitlServer;

        public string RemoteHost = "127.0.0.1";
        public int Px4TcpPort = NetworkConstants.Px4SitlTcpPort;
        public int RemotePort = NetworkConstants.QgcTelemetryUdpPort;
        public int LocalPort = NetworkConstants.LocalAutopilotUdpPort;

        public byte SystemId = 1;
        public byte ComponentId = 1;

        [Tooltip("Synchronizes Unity physics steps with PX4 SITL (HIL_SENSOR -> wait for HIL_ACTUATOR_CONTROLS).")]
        public bool EnableLockstepForSitl = true;

        [Range(2, 50)]
        public int LockstepTimeoutMs = 15;

        [Tooltip("Optional UDP port receiving MAVLink v2 ATTITUDE_TARGET (#83) / POSITION_TARGET_LOCAL_NED (#85) setpoints from ROS 2 / PX4 bridge.")]
        public int SetpointUdpPort = 8891;

        private readonly MavlinkSimulationBridge _bridge = new MavlinkSimulationBridge();
        private readonly ConcurrentQueue<byte[]> _rxQueue = new ConcurrentQueue<byte[]>();
        private readonly AutoResetEvent _rxPacketSignal = new AutoResetEvent(false);
        private readonly byte[] _udpSendBuffer = new byte[512];

        private UdpClient? _udpClient;
        private UdpClient? _setpointUdpClient;
        private IPEndPoint? _udpRemoteEndPoint;
        private TcpListener? _tcpServer;
        private TcpClient? _tcpClient;
        private NetworkStream? _tcpStream;
        private Thread? _ioThread;
        private Thread? _setpointThread;
        private volatile bool _running;

        public MavlinkSimulationBridge Bridge => _bridge;
        public bool IsConnected { get; private set; }

        private void Awake()
        {
            Application.runInBackground = true;
            if (Vehicle == null)
                Vehicle = GetComponent<FlightDynamicsVehicle>();
        }

        private void OnEnable()
        {
            StartNetworkTransport();
        }

        private void OnDisable()
        {
            StopNetworkTransport();
        }

        public void StartNetworkTransport()
        {
            StopNetworkTransport();

            if (Transport == NetworkTransportType.TcpPx4SitlServer || Transport == NetworkTransportType.TcpPx4SitlClient)
            {
                BridgeMode = MavlinkBridgeMode.ExternalSitlHil;
            }

            _bridge.Mode = BridgeMode;
            _bridge.SystemId = SystemId;
            _bridge.ComponentId = ComponentId;
            _running = true;

            try
            {
                if (Transport == NetworkTransportType.TcpPx4SitlServer)
                {
                    _tcpServer = new TcpListener(IPAddress.Any, Px4TcpPort);
                    _tcpServer.Server.SetSocketOption(SocketOptionLevel.Socket, SocketOptionName.ReuseAddress, true);
                    _tcpServer.Start();
                    _ioThread = new Thread(TcpServerLoop) { IsBackground = true, Name = "FD6DoF_PX4_TCP_Server_4560" };
                    _ioThread.Start();
                }
                else if (Transport == NetworkTransportType.TcpPx4SitlClient)
                {
                    _ioThread = new Thread(TcpClientLoop) { IsBackground = true, Name = "FD6DoF_PX4_TCP_Client_4560" };
                    _ioThread.Start();
                }
                else
                {
                    _udpClient = new UdpClient(LocalPort);
                    _udpClient.Client.ReceiveTimeout = 50;
                    _udpRemoteEndPoint = new IPEndPoint(IPAddress.Parse(RemoteHost), RemotePort);
                    IsConnected = true;
                    _ioThread = new Thread(UdpReceiveLoop) { IsBackground = true, Name = "FD6DoF_MAVLink_UDP" };
                    _ioThread.Start();
                }

                if (SetpointUdpPort > 0)
                {
                    _setpointUdpClient = new UdpClient(SetpointUdpPort);
                    _setpointUdpClient.Client.ReceiveTimeout = 50;
                    _setpointThread = new Thread(SetpointUdpLoop) { IsBackground = true, Name = "FD6DoF_PX4_Setpoint_UDP" };
                    _setpointThread.Start();
                }
            }
            catch (Exception ex)
            {
                Debug.LogWarning($"[FlightDynamics6DoF] MAVLink transport start warning: {ex.Message}");
            }
        }

        public void StopNetworkTransport()
        {
            _running = false;
            IsConnected = false;

            try { _udpClient?.Close(); } catch { }
            try { _setpointUdpClient?.Close(); } catch { }
            try { _tcpStream?.Close(); } catch { }
            try { _tcpClient?.Close(); } catch { }
            try { _tcpServer?.Stop(); } catch { }

            _udpClient = null;
            _setpointUdpClient = null;
            _tcpStream = null;
            _tcpClient = null;
            _tcpServer = null;
        }

        private void FixedUpdate()
        {
            if (Vehicle == null || Vehicle.Engine == null || Vehicle.FlightController == null)
                return;

            if (Transport == NetworkTransportType.TcpPx4SitlServer || Transport == NetworkTransportType.TcpPx4SitlClient)
            {
                BridgeMode = MavlinkBridgeMode.ExternalSitlHil;
            }

            _bridge.Mode = BridgeMode;

            if (BridgeMode == MavlinkBridgeMode.ExternalSitlHil)
            {
                // In External SITL mode, PX4 is the sole controller; disable Unity's internal fallback controller
                Vehicle.UseInternalFlightController = false;

                // 1. Drain any already-arrived packets from PX4
                DrainRxQueue();

                // 2. Send current environment sensor state (HIL_SENSOR, HIL_STATE_QUATERNION, HIL_GPS) to PX4
                if (IsConnected)
                {
                    _bridge.StepOutboundTelemetry(Vehicle.Engine, Vehicle.LatestSensorReadings, Vehicle.FlightController, SendRawBytes);

                    // 3. If Lockstep is enabled and PX4 has finished booting (started sending HIL_ACTUATOR_CONTROLS),
                    // wait up to LockstepTimeoutMs for PX4's actuator output for this step
                    if (EnableLockstepForSitl && _bridge.TotalHilActuatorFramesReceived > 0 && !_bridge.ReceivedActuatorControlsThisStep)
                    {
                        var sw = Stopwatch.StartNew();
                        while (!_bridge.ReceivedActuatorControlsThisStep && sw.ElapsedMilliseconds < LockstepTimeoutMs)
                        {
                            _rxPacketSignal.WaitOne(2);
                            DrainRxQueue();
                        }
                    }
                }
            }
            else
            {
                DrainRxQueue();
                _bridge.StepOutboundTelemetry(Vehicle.Engine, Vehicle.LatestSensorReadings, Vehicle.FlightController, SendRawBytes);
            }
        }

        private void DrainRxQueue()
        {
            if (Vehicle == null || Vehicle.Engine == null || Vehicle.FlightController == null)
                return;

            while (_rxQueue.TryDequeue(out byte[]? rawBytes))
            {
                _bridge.ProcessIncomingBytes(rawBytes, Vehicle.Engine, Vehicle.FlightController, SendRawBytes);
            }

            if (!Vehicle.UseInternalFlightController && _bridge.HasExternalTargetSetpoint)
            {
                _bridge.ApplyExternalSetpointsToLogger(Vehicle.TelemetryLogger);
            }
            else if (Vehicle.UseInternalFlightController && Vehicle.TelemetryLogger.HasExternalTargetSetpoint)
            {
                Vehicle.TelemetryLogger.ClearExternalTargetSetpoint();
            }
        }

        private void SetpointUdpLoop()
        {
            var ep = new IPEndPoint(IPAddress.Any, 0);
            while (_running && _setpointUdpClient != null)
            {
                try
                {
                    byte[] data = _setpointUdpClient.Receive(ref ep);
                    if (data.Length > 0)
                        _rxQueue.Enqueue(data);
                }
                catch (SocketException)
                {
                }
                catch
                {
                    if (!_running) break;
                    Thread.Sleep(5);
                }
            }
        }

        private void SendRawBytes(ReadOnlyMemory<byte> packet)
        {
            if (!_running) return;

            try
            {
                if (Transport == NetworkTransportType.UdpGcsOrSitl && _udpClient != null && _udpRemoteEndPoint != null)
                {
                    int len = packet.Length;
                    if (len <= _udpSendBuffer.Length)
                    {
                        packet.Span.CopyTo(_udpSendBuffer);
                        _udpClient.Client.SendTo(_udpSendBuffer, 0, len, SocketFlags.None, _udpRemoteEndPoint);
                    }
                }
                else if (_tcpStream != null && _tcpStream.CanWrite)
                {
                    _tcpStream.Write(packet.Span);
                }
            }
            catch
            {
                // Ignore transient socket disconnects
            }
        }

        private void UdpReceiveLoop()
        {
            IPEndPoint anyEp = new IPEndPoint(IPAddress.Any, 0);
            while (_running && _udpClient != null)
            {
                try
                {
                    byte[] data = _udpClient.Receive(ref anyEp);
                    if (data != null && data.Length > 0)
                    {
                        _rxQueue.Enqueue(data);
                        _rxPacketSignal.Set();
                    }
                }
                catch (SocketException)
                {
                }
                catch
                {
                    break;
                }
            }
        }

        private void TcpServerLoop()
        {
            byte[] buffer = new byte[1024];
            while (_running && _tcpServer != null)
            {
                try
                {
                    if (!_tcpServer.Pending())
                    {
                        Thread.Sleep(5);
                        continue;
                    }

                    try { _tcpStream?.Close(); } catch { }
                    try { _tcpClient?.Close(); } catch { }

                    _tcpClient = _tcpServer.AcceptTcpClient();
                    _tcpClient.NoDelay = true;
                    _tcpClient.ReceiveTimeout = 250;
                    _tcpClient.SendTimeout = 250;
                    _tcpStream = _tcpClient.GetStream();
                    IsConnected = true;
                    Debug.Log($"[FlightDynamics6DoF] External PX4 SITL connected to TCP port {Px4TcpPort}!");

                    // Immediately send an initial HIL_SENSOR frame so PX4 SimulatorMavlink activates lockstep stream
                    if (Vehicle != null && Vehicle.Engine != null && Vehicle.FlightController != null)
                    {
                        _bridge.StepOutboundTelemetry(Vehicle.Engine, Vehicle.LatestSensorReadings, Vehicle.FlightController, SendRawBytes);
                    }

                    while (_running && _tcpClient.Connected)
                    {
                        // Preempt stale connection if a newly restarted container is waiting to connect
                        if (_tcpServer.Pending())
                            break;

                        try
                        {
                            int n = _tcpStream.Read(buffer, 0, buffer.Length);
                            if (n <= 0) break;
                            byte[] chunk = new byte[n];
                            Buffer.BlockCopy(buffer, 0, chunk, 0, n);
                            _rxQueue.Enqueue(chunk);
                            _rxPacketSignal.Set();
                        }
                        catch (IOException ioEx) when (ioEx.InnerException is SocketException sockEx &&
                                                       (sockEx.SocketErrorCode == SocketError.TimedOut ||
                                                        sockEx.SocketErrorCode == SocketError.WouldBlock))
                        {
                            // Normal read timeout while PX4 is booting; loop to check _running and _tcpServer.Pending()
                            continue;
                        }
                    }
                }
                catch
                {
                    Thread.Sleep(25);
                }
                finally
                {
                    IsConnected = false;
                }
            }
        }

        private void TcpClientLoop()
        {
            byte[] buffer = new byte[1024];
            while (_running)
            {
                try
                {
                    _tcpClient = new TcpClient();
                    _tcpClient.Connect(RemoteHost, Px4TcpPort);
                    _tcpClient.NoDelay = true;
                    _tcpStream = _tcpClient.GetStream();
                    IsConnected = true;

                    while (_running && _tcpClient.Connected)
                    {
                        int n = _tcpStream.Read(buffer, 0, buffer.Length);
                        if (n <= 0) break;
                        byte[] chunk = new byte[n];
                        Buffer.BlockCopy(buffer, 0, chunk, 0, n);
                        _rxQueue.Enqueue(chunk);
                        _rxPacketSignal.Set();
                    }
                }
                catch
                {
                    IsConnected = false;
                    Thread.Sleep(250);
                }
            }
        }
    }
}
