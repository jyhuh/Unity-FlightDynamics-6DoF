// Licensed under the Apache License, Version 2.0.
#nullable enable
using System.Collections.Generic;
using FlightDynamics6DoF.Core.Dynamics;
using UnityEditor;
using UnityEngine;

namespace FlightDynamics6DoF.Editor
{
    /// <summary>
    /// Shared horizontal time-axis viewport state for synchronized pan, zoom, and live-tail tracking across charts.
    /// </summary>
    internal sealed class TelemetryTimeViewport
    {
        public const float DefaultWindowSec = 30.0f;
        public const float MinWindowSec = 1.0f;
        public const float MaxWindowSec = 300.0f;

        public float WindowSec { get; set; } = DefaultWindowSec;
        public bool FollowLiveTail { get; set; } = true;
        public double ViewEndTimeSec { get; private set; } = DefaultWindowSec;

        private bool _isPanning;
        private float _panLastMouseX;

        public void Reset(double tLast)
        {
            WindowSec = DefaultWindowSec;
            FollowLiveTail = true;
            ViewEndTimeSec = tLast;
            _isPanning = false;
        }

        public void SetViewEndTime(double timeSec, double minEnd, double tLast)
        {
            ViewEndTimeSec = System.Math.Clamp(timeSec, minEnd, tLast);
            FollowLiveTail = false;
        }

        public (double tMin, double tMax, double minEnd, double tLast) SyncWithFrames(IReadOnlyList<TelemetryLogFrame> frames)
        {
            double tFirst = frames[0].SimTimeSec;
            double tLast = frames[frames.Count - 1].SimTimeSec;
            double minEnd = System.Math.Min(tLast, tFirst + System.Math.Min(WindowSec, tLast - tFirst));
            if (FollowLiveTail)
                ViewEndTimeSec = tLast;
            ViewEndTimeSec = System.Math.Clamp(ViewEndTimeSec, minEnd, tLast);
            double tMax = FollowLiveTail ? tLast : System.Math.Clamp(ViewEndTimeSec, tFirst, tLast);
            double tMin = System.Math.Max(tFirst, tMax - WindowSec);
            return (tMin, tMax, minEnd, tLast);
        }

        public void HandlePlotInput(Rect plotRect, Event ev, EditorWindow host, double tMin, double tMax, double minEnd, double tLast)
        {
            double dt = System.Math.Max(1e-3, tMax - tMin);

            if (plotRect.Contains(ev.mousePosition))
            {
                if (ev.type == EventType.ScrollWheel)
                {
                    float factor = ev.delta.y > 0 ? 1.15f : 0.87f;
                    WindowSec = Mathf.Clamp(WindowSec * factor, MinWindowSec, MaxWindowSec);
                    ev.Use();
                    host.Repaint();
                }
                else if (ev.type == EventType.MouseDown && ev.button == 0)
                {
                    _isPanning = true;
                    _panLastMouseX = ev.mousePosition.x;
                    FollowLiveTail = false;
                    ev.Use();
                }
            }

            if (_isPanning && ev.type == EventType.MouseDrag)
            {
                float dx = ev.mousePosition.x - _panLastMouseX;
                _panLastMouseX = ev.mousePosition.x;
                double shiftSec = (dx / Mathf.Max(1f, plotRect.width)) * dt;
                ViewEndTimeSec = System.Math.Clamp(tMax - shiftSec, minEnd, tLast);
                ev.Use();
                host.Repaint();
            }
            else if (_isPanning && ev.type == EventType.MouseUp)
            {
                _isPanning = false;
            }
        }

        public static (int startIdx, int endIdx, int stride) ResolveSampleRange(
            IReadOnlyList<TelemetryLogFrame> frames, double tMin, double tMax, int maxSamples = 280)
        {
            int n = frames.Count;
            int startIdx = 0;
            for (int i = n - 1; i >= 0; i--)
            {
                if (frames[i].SimTimeSec <= tMin) { startIdx = i; break; }
            }

            int endIdx = n - 1;
            for (int i = n - 1; i >= startIdx; i--)
            {
                if (frames[i].SimTimeSec <= tMax) { endIdx = i; break; }
            }

            int count = System.Math.Max(1, endIdx - startIdx + 1);
            int stride = System.Math.Max(1, count / maxSamples);
            return (startIdx, endIdx, stride);
        }
    }
}
