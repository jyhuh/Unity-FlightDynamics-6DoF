// Licensed under the Apache License, Version 2.0.
#nullable enable
using System.Collections.Generic;
using FlightDynamics6DoF.Core.Dynamics;
using UnityEditor;
using UnityEngine;

namespace FlightDynamics6DoF.Editor
{
    /// <summary>
    /// Stateless IMGUI renderer for <see cref="TelemetryChartDefinition"/> bound to a shared <see cref="TelemetryTimeViewport"/>.
    /// </summary>
    internal static class TelemetryChartRenderer
    {
        public static void Draw(
            EditorWindow host,
            TelemetryChartDefinition chart,
            IReadOnlyList<TelemetryLogFrame> frames,
            TelemetryTimeViewport viewport,
            (double tMin, double tMax, double minEnd, double tLast) bounds)
        {
            if (chart.Series.Count == 0 || frames.Count < 2)
                return;

            EditorGUILayout.BeginVertical(EditorStyles.helpBox);
            DrawHeaderLegend(chart);

            Rect plotRect = GUILayoutUtility.GetRect(100, 130, GUILayout.ExpandWidth(true));
            Event ev = Event.current;
            var (tMin, tMax, minEnd, tLast) = bounds;
            viewport.HandlePlotInput(plotRect, ev, host, tMin, tMax, minEnd, tLast);

            if (ev.type == EventType.Repaint || plotRect.Contains(ev.mousePosition))
            {
                DrawPlotSurface(host, plotRect, ev, chart, frames, tMin, tMax);
            }

            EditorGUILayout.EndVertical();
        }

        private static void DrawHeaderLegend(TelemetryChartDefinition chart)
        {
            EditorGUILayout.BeginHorizontal();
            GUILayout.Label(chart.Title, EditorStyles.boldLabel);
            GUILayout.FlexibleSpace();
            for (int s = 0; s < chart.Series.Count; s++)
            {
                var prev = GUI.contentColor;
                GUI.contentColor = chart.Series[s].Color;
                GUILayout.Label($"■ {chart.Series[s].Name}", EditorStyles.miniBoldLabel);
                GUI.contentColor = prev;
            }
            EditorGUILayout.EndHorizontal();
        }

        private static void DrawPlotSurface(
            EditorWindow host,
            Rect plotRect,
            Event ev,
            TelemetryChartDefinition chart,
            IReadOnlyList<TelemetryLogFrame> frames,
            double tMin,
            double tMax)
        {
            EditorGUI.DrawRect(plotRect, new Color(0.08f, 0.10f, 0.13f, 1f));
            var (startIdx, endIdx, stride) = TelemetryTimeViewport.ResolveSampleRange(frames, tMin, tMax);
            var (minY, maxY) = ComputeYRange(chart.Series, frames, startIdx, endIdx, stride);

            Color gridCol = new Color(1f, 1f, 1f, 0.08f);
            for (int g = 1; g <= 3; g++)
            {
                float yG = plotRect.y + (g / 4f) * plotRect.height;
                EditorGUI.DrawRect(new Rect(plotRect.x, yG, plotRect.width, 1f), gridCol);
            }

            double dt = System.Math.Max(1e-3, tMax - tMin);
            for (int s = 0; s < chart.Series.Count; s++)
            {
                var series = chart.Series[s];
                var pts = new List<Vector3>();
                for (int i = startIdx; i <= endIdx; i += stride)
                {
                    var f = frames[i];
                    float x = plotRect.x + (float)((f.SimTimeSec - tMin) / dt) * plotRect.width;
                    double clamped = System.Math.Clamp(series.Extractor(f), minY, maxY);
                    float y = plotRect.yMax - (float)((clamped - minY) / (maxY - minY)) * plotRect.height;
                    pts.Add(new Vector3(x, y, 0f));
                }
                if (pts.Count >= 2)
                {
                    Handles.color = series.Color;
                    Handles.DrawAAPolyLine(2.0f, pts.ToArray());
                }
            }

            GUI.Label(new Rect(plotRect.x + 4, plotRect.y + 2, 120, 16), $"{maxY:F2}", EditorStyles.miniLabel);
            GUI.Label(new Rect(plotRect.x + 4, plotRect.yMax - 16, 80, 16), $"{minY:F2}", EditorStyles.miniLabel);
            GUI.Label(new Rect(plotRect.x + 64, plotRect.yMax - 16, 80, 16), $"t={tMin:F1}s", EditorStyles.miniLabel);
            GUI.Label(new Rect(plotRect.xMax - 72, plotRect.yMax - 16, 68, 16), $"t={tMax:F1}s", EditorStyles.miniLabel);

            if (plotRect.Contains(ev.mousePosition))
            {
                DrawHoverReadout(host, plotRect, ev.mousePosition.x, chart.Series, frames, startIdx, endIdx, stride, tMin, dt);
            }
        }

        private static (double minY, double maxY) ComputeYRange(
            IReadOnlyList<TelemetryChannelDef> series,
            IReadOnlyList<TelemetryLogFrame> frames,
            int startIdx,
            int endIdx,
            int stride)
        {
            double minY = double.PositiveInfinity;
            double maxY = double.NegativeInfinity;

            for (int i = startIdx; i <= endIdx; i += stride)
            {
                var f = frames[i];
                for (int s = 0; s < series.Count; s++)
                {
                    double v = series[s].Extractor(f);
                    if (!double.IsNaN(v) && !double.IsInfinity(v))
                    {
                        if (v < minY) minY = v;
                        if (v > maxY) maxY = v;
                    }
                }
            }

            if (double.IsInfinity(minY) || double.IsInfinity(maxY)) return (-1.0, 1.0);
            if (System.Math.Abs(maxY - minY) < 1e-3) return (minY - 0.5, maxY + 0.5);
            double pad = 0.08 * (maxY - minY);
            return (minY - pad, maxY + pad);
        }

        private static void DrawHoverReadout(
            EditorWindow host,
            Rect plotRect,
            float mouseX,
            IReadOnlyList<TelemetryChannelDef> series,
            IReadOnlyList<TelemetryLogFrame> frames,
            int startIdx,
            int endIdx,
            int stride,
            double tMin,
            double dt)
        {
            float mouseFrac = (mouseX - plotRect.x) / Mathf.Max(1f, plotRect.width);
            double hoverTime = tMin + mouseFrac * dt;
            int nearestIdx = startIdx;
            double bestDiff = double.MaxValue;
            for (int i = startIdx; i <= endIdx; i += stride)
            {
                double d = System.Math.Abs(frames[i].SimTimeSec - hoverTime);
                if (d < bestDiff) { bestDiff = d; nearestIdx = i; }
            }

            EditorGUI.DrawRect(new Rect(mouseX, plotRect.y, 1f, plotRect.height), new Color(1f, 1f, 1f, 0.35f));
            var hf = frames[nearestIdx];
            string readout = $"t={hf.SimTimeSec:F2}s:";
            for (int s = 0; s < series.Count; s++)
                readout += $" {series[s].Name}={series[s].Extractor(hf):F2}";

            GUI.Label(new Rect(plotRect.x + 80, plotRect.y + 2, plotRect.width - 84, 18), readout, EditorStyles.miniBoldLabel);
            host.Repaint();
        }
    }
}
