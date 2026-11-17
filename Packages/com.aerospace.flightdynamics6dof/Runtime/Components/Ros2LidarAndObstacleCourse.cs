// Licensed under the Apache License, Version 2.0.
using System;
using System.Collections.Generic;
using System.Net;
using System.Net.Sockets;
using UnityEngine;

namespace FlightDynamics6DoF.Unity
{
    /// <summary>
    /// Lightweight 2D horizontal Raycast LiDAR + UDP streamer for ROS 2 Nav2 (`/scan`),
    /// with optional procedural obstacle course generation for autonomous avoidance testing.
    /// </summary>
    public class Ros2LidarAndObstacleCourse : MonoBehaviour
    {
        [Header("2D Raycast LiDAR (/scan)")]
        public bool EnableLidar = true;
        public int UdpListenPort = 8890;
        public float ScanRateHz = 10.0f;
        [Range(36, 360)]
        public int RayCount = 180;
        public float MinRangeMeters = 0.4f;
        public float MaxRangeMeters = 20.0f;

        [Header("Test Obstacle Course")]
        public bool GenerateTestObstacles = true;

        public int ActiveSubscribers => _remoteEndPoint != null ? 1 : 0;
        public int TotalScansSent { get; private set; }

        private UdpClient? _udp;
        private IPEndPoint? _remoteEndPoint;
        private readonly HashSet<Collider> _selfColliders = new HashSet<Collider>();
        private float[] _ranges = Array.Empty<float>();
        private byte[] _packetBuf = Array.Empty<byte>();
        private float _nextScanTime;
        private GameObject? _obstacleRoot;

        private void Start()
        {
            foreach (var col in GetComponentsInChildren<Collider>(true))
                _selfColliders.Add(col);

            if (GenerateTestObstacles)
                EnsureTestObstacleCourse();

            StartUdpListener();
        }

        private void OnDestroy()
        {
            try { _udp?.Close(); } catch { }
            _udp = null;
            if (_obstacleRoot != null)
            {
                Destroy(_obstacleRoot);
                _obstacleRoot = null;
            }
        }

        private void StartUdpListener()
        {
            try
            {
                _udp = new UdpClient(UdpListenPort);
                _udp.Client.Blocking = false;
                Debug.Log($"[FlightDynamics6DoF] ROS 2 LiDAR UDP bridge listening on :{UdpListenPort}");
            }
            catch (Exception ex)
            {
                Debug.LogWarning($"[FlightDynamics6DoF] Could not bind ROS 2 LiDAR UDP port {UdpListenPort}: {ex.Message}");
            }
        }

        private void FixedUpdate()
        {
            if (!EnableLidar || _udp == null) return;

            PollSubscriber();

            if (Time.time < _nextScanTime) return;
            _nextScanTime = Time.time + 1.0f / Mathf.Max(1.0f, ScanRateHz);

            if (_remoteEndPoint == null) return;

            PerformScanAndSend();
        }

        private void PollSubscriber()
        {
            if (_udp == null) return;
            try
            {
                while (_udp.Available > 0)
                {
                    IPEndPoint sender = new IPEndPoint(IPAddress.Any, 0);
                    byte[] data = _udp.Receive(ref sender);
                    if (data != null && data.Length > 0)
                        _remoteEndPoint = sender;
                }
            }
            catch
            {
                // Non-blocking socket poll
            }
        }

        private void PerformScanAndSend()
        {
            int n = Mathf.Clamp(RayCount, 36, 360);
            if (_ranges.Length != n)
            {
                _ranges = new float[n];
                _packetBuf = new byte[14 + n * 4];
            }

            Vector3 origin = transform.position;
            Vector3 fwd = Vector3.ProjectOnPlane(transform.forward, Vector3.up);
            if (fwd.sqrMagnitude < 1e-4f) fwd = Vector3.forward;
            fwd.Normalize();
            // Unity Left-Handed: Cross(Forward, Up) = Left (+Y in ROS FLU)
            Vector3 left = Vector3.Cross(fwd, Vector3.up).normalized;

            float angleInc = (2.0f * Mathf.PI) / n;
            for (int i = 0; i < n; i++)
            {
                float angle = i * angleInc;
                Vector3 dir = fwd * Mathf.Cos(angle) + left * Mathf.Sin(angle);

                float dist = float.PositiveInfinity;
                if (Physics.Raycast(origin, dir, out RaycastHit hit, MaxRangeMeters, ~0, QueryTriggerInteraction.Ignore))
                {
                    if (!_selfColliders.Contains(hit.collider) && hit.distance >= MinRangeMeters)
                        dist = hit.distance;
                }
                _ranges[i] = dist;
            }

            // Packet format: [magic "SCAN" (4B)] [uint16 n] [float32 minRange] [float32 maxRange] [float32[n] ranges]
            _packetBuf[0] = (byte)'S';
            _packetBuf[1] = (byte)'C';
            _packetBuf[2] = (byte)'A';
            _packetBuf[3] = (byte)'N';
            _packetBuf[4] = (byte)(n & 0xFF);
            _packetBuf[5] = (byte)((n >> 8) & 0xFF);
            Buffer.BlockCopy(BitConverter.GetBytes(MinRangeMeters), 0, _packetBuf, 6, 4);
            Buffer.BlockCopy(BitConverter.GetBytes(MaxRangeMeters), 0, _packetBuf, 10, 4);
            Buffer.BlockCopy(_ranges, 0, _packetBuf, 14, n * 4);

            try
            {
                _udp?.Send(_packetBuf, _packetBuf.Length, _remoteEndPoint);
                TotalScansSent++;
            }
            catch
            {
                // Ignore transient UDP send errors
            }
        }

        private void EnsureTestObstacleCourse()
        {
            if (GameObject.Find("UnityNav2ObstacleCourse") != null) return;

            _obstacleRoot = new GameObject("UnityNav2ObstacleCourse");
            Vector3 center = new Vector3(transform.position.x, 0.0f, transform.position.z);

            // Tall pillars & walls (height 6m) so 2D LiDAR at hover altitude (~2.5m) reliably detects them
            (Vector3 offset, Vector3 scale)[] layout = new[]
            {
                (new Vector3( 0.0f, 3.0f,  6.0f), new Vector3(1.6f, 6.0f, 1.6f)),
                (new Vector3( 4.0f, 3.0f, 10.0f), new Vector3(1.8f, 6.0f, 1.8f)),
                (new Vector3(-4.5f, 3.0f,  9.5f), new Vector3(1.8f, 6.0f, 1.8f)),
                (new Vector3( 1.0f, 3.0f, 14.5f), new Vector3(4.0f, 6.0f, 1.2f)),
                (new Vector3(-7.0f, 3.0f,  4.0f), new Vector3(1.5f, 6.0f, 3.0f)),
                (new Vector3( 7.5f, 3.0f,  3.5f), new Vector3(1.5f, 6.0f, 3.0f)),
                (new Vector3(-2.5f, 3.0f, -6.5f), new Vector3(2.0f, 6.0f, 1.5f)),
                (new Vector3( 5.0f, 3.0f, -5.5f), new Vector3(1.5f, 6.0f, 2.0f)),
            };

            var mat = new Material(Shader.Find("Standard") ?? Shader.Find("Universal Render Pipeline/Lit"));
            mat.color = new Color(0.85f, 0.35f, 0.15f, 1.0f);

            for (int i = 0; i < layout.Length; i++)
            {
                var box = GameObject.CreatePrimitive(PrimitiveType.Cube);
                box.name = $"Obstacle_{i + 1}";
                box.transform.SetParent(_obstacleRoot.transform, false);
                box.transform.position = center + layout[i].offset;
                box.transform.localScale = layout[i].scale;
                var renderer = box.GetComponent<Renderer>();
                if (renderer != null && mat != null) renderer.sharedMaterial = mat;
            }
        }
    }
}

