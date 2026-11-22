// Licensed under the Apache License, Version 2.0.
#nullable enable
using System;
using System.Collections.Generic;
using FlightDynamics6DoF.Core.Aerodynamics;
using FlightDynamics6DoF.Core.Math;
using FlightDynamics6DoF.Core.Specs;
using FlightDynamics6DoF.Unity;
using UnityEditor;
using UnityEngine;

namespace FlightDynamics6DoF.Editor
{
    /// <summary>
    /// Interactive Airframe Shape, Dimension & Spec Designer popup launched from Step 1.
    /// Visualizes Multirotor, Fixed-Wing, and Hybrid VTOL geometry in a live Top/Side CAD blueprint
    /// while editing dimensions, mass/inertia, motor power, battery specs, and engineering notes.
    /// </summary>
    internal sealed class AirframeDesignerWindow : EditorWindow
    {
        private FlightDynamicsEngineeringWizard? _host;
        private AircraftConfigSpec _workingSpec = AircraftConfigSpec.CreateStandardQuadcopterX();
        private Vector2 _rightScrollPos;
        private double _cachedMaxThrustN;
        private double _cachedShaftPowerW;
        private bool _metricsDirty = true;

        // Module 4: BEMT Rotor Solver state
        private bool _showAeroSolvers;
        private float _testRotorRpm = 6500f;
        private float _testRotorAxialSpeedMps = 0.0f;
        private float _testRotorAglMeters = 10.0f;
        private RotorBemtSolution _lastRotorSolution;
        private bool _hasRotorResult;

        // Module 1: 2D Airfoil Polar Solver state
        private string _nacaCode = "2412";
        private float _testAlphaDeg = 5.0f;
        private float _testReynolds = 500000f;
        private float _testMach = 0.08f;
        private AirfoilPolarPoint _lastAirfoilPoint;
        private bool _hasAirfoilResult;

        private static readonly Vector2 MinWindowSize = new Vector2(960f, 680f);

        public static void Open(FlightDynamicsEngineeringWizard host)
        {
            var win = GetWindow<AirframeDesignerWindow>(true, "Drone / Fixed-Wing / VTOL Shape & Spec Designer", true);
            win.minSize = MinWindowSize;
            win._host = host;
            win.LoadFromHost();
            win.Show();
            win.Focus();
        }

        private void OnEnable()
        {
            minSize = MinWindowSize;
        }

        private void LoadFromHost()
        {
            if (_host == null) return;
            var src = _host.GetCurrentConfig();
            string json = JsonUtility.ToJson(src);
            var clone = JsonUtility.FromJson<AircraftConfigSpec>(json);
            _workingSpec = clone ?? AircraftConfigSpec.CreateStandardQuadcopterX();
            _metricsDirty = true;
        }

        private void OnGUI()
        {
            if (_metricsDirty)
            {
                RecomputePropulsionMetrics();
                _metricsDirty = false;
            }

            EditorGUILayout.BeginHorizontal();

            // Left Pane: Live Top + Side CAD Blueprint Preview
            EditorGUILayout.BeginVertical(GUILayout.Width(430f), GUILayout.ExpandHeight(true));
            DrawBlueprintPreview(_workingSpec, 400f, 520f, expandHeight: true);
            EditorGUILayout.EndVertical();

            // Right Pane: Dimensions, Weight, Motor Power, Battery & Notes
            EditorGUILayout.BeginVertical(GUILayout.ExpandWidth(true), GUILayout.ExpandHeight(true));
            DrawSpecEditorPane();
            EditorGUILayout.EndVertical();

            EditorGUILayout.EndHorizontal();
        }

        internal static void DrawBlueprintPreview(AircraftConfigSpec spec, float minWidth, float minHeight, bool expandHeight)
        {
            EditorGUILayout.BeginVertical(EditorStyles.helpBox, GUILayout.ExpandHeight(expandHeight));
            GUILayout.Label($"Live Airframe Blueprint — {spec.Category}", EditorStyles.boldLabel);

            Rect canvas = GUILayoutUtility.GetRect(minWidth, minHeight, GUILayout.ExpandWidth(true), GUILayout.ExpandHeight(expandHeight));
            EditorGUI.DrawRect(canvas, new Color(0.08f, 0.11f, 0.16f, 1f));

            if (Event.current.type == EventType.Repaint)
            {
                DrawGrid(canvas, 24f, new Color(0.16f, 0.22f, 0.30f, 0.5f));

                Rect topRect = new Rect(canvas.x + 8f, canvas.y + 8f, canvas.width - 16f, canvas.height * 0.56f - 12f);
                Rect sideRect = new Rect(canvas.x + 8f, canvas.y + canvas.height * 0.56f + 4f, canvas.width - 16f, canvas.height * 0.44f - 12f);

                DrawTopView(spec, topRect);
                DrawSideView(spec, sideRect);
            }

            // Blueprint Legend
            EditorGUILayout.LabelField(
                $"Scale: Auto-fit | Fuselage: {spec.Fuselage.Length:F2}m × Ø{spec.Fuselage.MaxDiameter:F2}m | Gear H: {spec.LandingGearHeightMeters:F2}m",
                EditorStyles.miniLabel);
            EditorGUILayout.EndVertical();
        }

        private static void DrawTopView(AircraftConfigSpec spec, Rect rect)
        {
            GUI.Label(new Rect(rect.x + 4f, rect.y + 2f, 220f, 18f), "TOP-DOWN PLANFORM (Nose ▲ +X)", EditorStyles.whiteMiniLabel);

            float extentMeters = ComputeBlueprintExtentMeters(spec);
            float pxPerM = Mathf.Min(rect.width, rect.height - 24f) / (extentMeters * 2.25f);
            Vector2 center = new Vector2(rect.x + rect.width * 0.5f, rect.y + rect.height * 0.54f);

            // Helper: FRD (X=Forward, Y=Right) -> Screen (Right=+Y, Up=+X)
            Vector2 ToTopPt(double frdX, double frdY) =>
                new Vector2(center.x + (float)frdY * pxPerM, center.y - (float)frdX * pxPerM);

            Handles.BeginGUI();

            // 1. Wings (Fixed-Wing & VTOL)
            foreach (var wing in spec.Wings)
            {
                if (wing.IsVerticalFin) continue;
                float halfSpan = (float)(wing.Span * 0.5);
                float rootC = (float)wing.RootChord;
                float tipC = (float)wing.TipChord;
                float apexX = (float)wing.ApexPositionFrd.X;
                float sweepTan = Mathf.Tan((float)(wing.SweepQuarterChordDeg * System.Math.PI / 180.0));
                float tipLeX = apexX - 0.25f * rootC - halfSpan * sweepTan + 0.25f * tipC;

                Vector3[] poly =
                {
                    ToTopPt(apexX, 0.0),
                    ToTopPt(tipLeX, halfSpan),
                    ToTopPt(tipLeX - tipC, halfSpan),
                    ToTopPt(apexX - rootC, 0.0),
                    ToTopPt(tipLeX - tipC, -halfSpan),
                    ToTopPt(tipLeX, -halfSpan)
                };
                Handles.color = new Color(0.28f, 0.56f, 0.85f, 0.30f);
                Handles.DrawAAConvexPolygon(poly[0], poly[1], poly[2], poly[3]);
                Handles.DrawAAConvexPolygon(poly[0], poly[3], poly[4], poly[5]);
                Handles.color = new Color(0.55f, 0.82f, 1.0f, 0.95f);
                for (int i = 0; i < poly.Length; i++)
                    Handles.DrawLine(poly[i], poly[(i + 1) % poly.Length]);
            }

            // 2. Fuselage Pod (matches ProceduralAircraftVisualBuilder 0.75 * Length)
            float fLen = (float)spec.Fuselage.Length * 0.75f * pxPerM;
            float fDia = Mathf.Max(8f, (float)spec.Fuselage.MaxDiameter * pxPerM);
            Rect fusRect = new Rect(center.x - fDia * 0.5f, center.y - fLen * 0.5f, fDia, fLen);
            EditorGUI.DrawRect(fusRect, new Color(0.22f, 0.26f, 0.34f, 0.95f));
            Handles.color = new Color(0.75f, 0.82f, 0.92f, 0.9f);
            Handles.DrawAAPolyLine(2f,
                new Vector3(fusRect.xMin, fusRect.yMin), new Vector3(fusRect.xMax, fusRect.yMin),
                new Vector3(fusRect.xMax, fusRect.yMax), new Vector3(fusRect.xMin, fusRect.yMax),
                new Vector3(fusRect.xMin, fusRect.yMin));

            // 3. Arms & Rotors
            foreach (var r in spec.Rotors)
            {
                Vector2 rPos = ToTopPt(r.MountPositionFrd.X, r.MountPositionFrd.Y);
                bool isVerticalLift = System.Math.Abs(r.ThrustAxisFrd.Z) >= System.Math.Abs(r.ThrustAxisFrd.X);
                if (isVerticalLift)
                {
                    Handles.color = new Color(0.55f, 0.62f, 0.72f, 0.9f);
                    Handles.DrawAAPolyLine(3f, center, rPos);

                    float radPx = Mathf.Max(6f, (float)r.Radius * pxPerM);
                    Color discCol = r.RotationDirection >= 0
                        ? new Color(1.0f, 0.58f, 0.20f, 0.85f)
                        : new Color(0.22f, 0.86f, 0.95f, 0.85f);
                    Handles.color = new Color(discCol.r, discCol.g, discCol.b, 0.16f);
                    Handles.DrawSolidDisc(rPos, Vector3.forward, radPx);
                    Handles.color = discCol;
                    Handles.DrawWireDisc(rPos, Vector3.forward, radPx);
                    Handles.DrawSolidDisc(rPos, Vector3.forward, 3.5f);

                    string dirTag = r.RotationDirection >= 0 ? "CW" : "CCW";
                    GUI.Label(new Rect(rPos.x - 16f, rPos.y - 8f, 36f, 16f), dirTag, EditorStyles.whiteMiniLabel);
                }
                else
                {
                    // Horizontal tractor/pusher propeller shown as a thick line perpendicular to X
                    float radPx = Mathf.Max(8f, (float)r.Radius * pxPerM);
                    Handles.color = new Color(1.0f, 0.72f, 0.25f, 0.95f);
                    Handles.DrawAAPolyLine(3f, new Vector3(rPos.x - radPx, rPos.y), new Vector3(rPos.x + radPx, rPos.y));
                }
            }

            // 4. CG Crosshair
            Vector2 cgPt = ToTopPt(spec.CenterOfGravityFrd.X, spec.CenterOfGravityFrd.Y);
            Handles.color = Color.yellow;
            Handles.DrawWireDisc(cgPt, Vector3.forward, 5f);
            Handles.DrawLine(cgPt + new Vector2(-8f, 0f), cgPt + new Vector2(8f, 0f));
            Handles.DrawLine(cgPt + new Vector2(0f, -8f), cgPt + new Vector2(0f, 8f));

            // 5. Dimension Callout (Span / Wheelbase)
            float spanM = spec.Category == VehicleAirframeCategory.Multirotor
                ? EstimateMultirotorWheelbaseMeters(spec)
                : (float)spec.ReferenceSpanMeters;
            float halfSpanPx = spanM * 0.5f * pxPerM;
            float dimY = rect.yMax - 18f;
            Handles.color = new Color(0.45f, 0.95f, 0.65f, 0.9f);
            Handles.DrawLine(new Vector2(center.x - halfSpanPx, dimY), new Vector2(center.x + halfSpanPx, dimY));
            string spanLabel = spec.Category == VehicleAirframeCategory.Multirotor
                ? $"Wheelbase: {spanM:F2} m"
                : $"Span b: {spanM:F2} m";
            GUI.Label(new Rect(center.x - 55f, dimY - 16f, 120f, 16f), spanLabel, EditorStyles.whiteMiniLabel);

            Handles.EndGUI();
        }

        private static void DrawSideView(AircraftConfigSpec spec, Rect rect)
        {
            GUI.Label(new Rect(rect.x + 4f, rect.y + 2f, 220f, 18f), "SIDE ELEVATION (Nose ▶ +X)", EditorStyles.whiteMiniLabel);

            float extentMeters = ComputeBlueprintExtentMeters(spec);
            float pxPerM = Mathf.Min(rect.width, rect.height - 16f) / (extentMeters * 1.9f);
            Vector2 center = new Vector2(rect.x + rect.width * 0.5f, rect.y + rect.height * 0.50f);

            // FRD (X=Forward, Z=Down) -> Screen (Right=+X, Down=+Z)
            Vector2 ToSidePt(double frdX, double frdZ) =>
                new Vector2(center.x + (float)frdX * pxPerM, center.y + (float)frdZ * pxPerM);

            Handles.BeginGUI();

            // Ground line at LandingGearHeightMeters
            float gearZ = (float)System.Math.Max(0.05, spec.LandingGearHeightMeters);
            float groundY = ToSidePt(0.0, gearZ).y;
            Handles.color = new Color(0.35f, 0.75f, 0.45f, 0.65f);
            Handles.DrawLine(new Vector2(rect.x + 12f, groundY), new Vector2(rect.xMax - 12f, groundY));

            // Fuselage side profile (matches ProceduralAircraftVisualBuilder 0.75 * Length)
            float fLen = (float)spec.Fuselage.Length * 0.75f * pxPerM;
            float fDia = Mathf.Max(8f, (float)spec.Fuselage.MaxDiameter * pxPerM * 0.75f);
            Rect fusRect = new Rect(center.x - fLen * 0.5f, center.y - fDia * 0.5f, fLen, fDia);
            EditorGUI.DrawRect(fusRect, new Color(0.24f, 0.28f, 0.36f, 0.95f));

            // Landing gear struts
            Handles.color = new Color(0.75f, 0.80f, 0.88f, 0.9f);
            Vector2 gearFront = ToSidePt(spec.Fuselage.Length * 0.22, gearZ);
            Vector2 gearRear = ToSidePt(-spec.Fuselage.Length * 0.22, gearZ);
            Handles.DrawLine(ToSidePt(spec.Fuselage.Length * 0.15, 0.0), gearFront);
            Handles.DrawLine(ToSidePt(-spec.Fuselage.Length * 0.15, 0.0), gearRear);
            Handles.DrawLine(gearFront, gearRear);

            // Vertical Fin (if present)
            foreach (var wing in spec.Wings)
            {
                if (!wing.IsVerticalFin) continue;
                Vector2 rootLe = ToSidePt(wing.ApexPositionFrd.X, 0.0);
                Vector2 rootTe = ToSidePt(wing.ApexPositionFrd.X - wing.RootChord, 0.0);
                Vector2 tipTe = ToSidePt(wing.ApexPositionFrd.X - wing.TipChord - 0.05, -wing.Span);
                Vector2 tipLe = ToSidePt(wing.ApexPositionFrd.X - 0.05, -wing.Span);
                Handles.color = new Color(0.35f, 0.65f, 0.95f, 0.45f);
                Handles.DrawAAConvexPolygon(rootLe, tipLe, tipTe, rootTe);
            }

            // Rotors side view
            foreach (var r in spec.Rotors)
            {
                Vector2 rPos = ToSidePt(r.MountPositionFrd.X, r.MountPositionFrd.Z);
                float radPx = Mathf.Max(6f, (float)r.Radius * pxPerM);
                bool isVerticalLift = System.Math.Abs(r.ThrustAxisFrd.Z) >= System.Math.Abs(r.ThrustAxisFrd.X);
                Handles.color = new Color(1.0f, 0.65f, 0.25f, 0.95f);
                if (isVerticalLift)
                    Handles.DrawAAPolyLine(2.5f, rPos + new Vector2(-radPx, 0f), rPos + new Vector2(radPx, 0f));
                else
                    Handles.DrawAAPolyLine(2.5f, rPos + new Vector2(0f, -radPx), rPos + new Vector2(0f, radPx));
            }

            // Length & Gear Height callout
            GUI.Label(new Rect(rect.x + 12f, groundY + 2f, 220f, 16f),
                $"Length: {spec.Fuselage.Length:F2} m | Gear H: {gearZ:F2} m", EditorStyles.whiteMiniLabel);

            Handles.EndGUI();
        }

        private void DrawSpecEditorPane()
        {
            _rightScrollPos = EditorGUILayout.BeginScrollView(_rightScrollPos);

            // 1. Airframe Type Selector
            EditorGUILayout.BeginVertical(EditorStyles.helpBox);
            GUILayout.Label("1. Airframe Category & Identity", EditorStyles.boldLabel);
            EditorGUILayout.BeginHorizontal();
            DrawCategoryButton("Multirotor (Quad)", VehicleAirframeCategory.Multirotor);
            DrawCategoryButton("Fixed-Wing", VehicleAirframeCategory.FixedWing);
            DrawCategoryButton("VTOL (QuadPlane)", VehicleAirframeCategory.HybridQuadPlaneVtol);
            EditorGUILayout.EndHorizontal();

            _workingSpec.AircraftName = EditorGUILayout.TextField("Aircraft Name", _workingSpec.AircraftName);
            EditorGUILayout.EndVertical();

            EditorGUILayout.Space(4);

            // 2. Dimensions
            EditorGUILayout.BeginVertical(EditorStyles.helpBox);
            GUILayout.Label("2. Airframe Dimensions [m / m²]", EditorStyles.boldLabel);

            EditorGUI.BeginChangeCheck();
            double oldFusLen = System.Math.Max(0.15, _workingSpec.Fuselage.Length);
            double newFusLen = System.Math.Max(0.15, EditorGUILayout.FloatField("Fuselage Length [m]", (float)oldFusLen));
            if (System.Math.Abs(newFusLen - oldFusLen) > 1e-4)
            {
                double lenRatio = newFusLen / oldFusLen;
                _workingSpec.Fuselage.Length = newFusLen;

                // Keep tailplanes and tractor/pusher propellers proportional so they never get buried in the fuselage
                foreach (var w in _workingSpec.Wings)
                {
                    if (w.IsVerticalFin || w.ApexPositionFrd.X < -0.12)
                    {
                        double scaledX = w.ApexPositionFrd.X * lenRatio;
                        double minAftX = -newFusLen * 0.42;
                        if (scaledX > minAftX) scaledX = minAftX;
                        w.ApexPositionFrd = new Vec3d(scaledX, w.ApexPositionFrd.Y, w.ApexPositionFrd.Z);
                    }
                }
                foreach (var r in _workingSpec.Rotors)
                {
                    if (System.Math.Abs(r.ThrustAxisFrd.X) > System.Math.Abs(r.ThrustAxisFrd.Z))
                    {
                        double scaledX = r.MountPositionFrd.X * lenRatio;
                        double minClearX = newFusLen * 0.40;
                        if (scaledX >= 0.0 && scaledX < minClearX) scaledX = minClearX;
                        else if (scaledX < 0.0 && scaledX > -minClearX) scaledX = -minClearX;
                        r.MountPositionFrd = new Vec3d(scaledX, r.MountPositionFrd.Y, r.MountPositionFrd.Z);
                    }
                }
            }
            _workingSpec.Fuselage.MaxDiameter = System.Math.Max(0.05, EditorGUILayout.FloatField("Fuselage Width/Dia [m]", (float)_workingSpec.Fuselage.MaxDiameter));
            _workingSpec.LandingGearHeightMeters = System.Math.Max(0.04, EditorGUILayout.FloatField("Landing Gear Height [m]", (float)_workingSpec.LandingGearHeightMeters));

            var liftRotors = GetVerticalLiftRotors();
            if (liftRotors.Count > 0)
            {
                float armX = Mathf.Abs((float)liftRotors[0].MountPositionFrd.X);
                float armY = Mathf.Abs((float)liftRotors[0].MountPositionFrd.Y);
                float newArmX = EditorGUILayout.Slider("Rotor Arm X Offset [m]", armX, 0.08f, 1.50f);
                float newArmY = EditorGUILayout.Slider("Rotor Arm Y Offset [m]", armY, 0.08f, 1.50f);
                float radius = EditorGUILayout.Slider("Lift Rotor Radius R [m]", (float)liftRotors[0].Radius, 0.05f, 0.80f);
                int blades = EditorGUILayout.IntSlider("Lift Rotor Blades", liftRotors[0].NumBlades, 2, 6);

                if (!Mathf.Approximately(newArmX, armX) || !Mathf.Approximately(newArmY, armY) ||
                    !Mathf.Approximately(radius, (float)liftRotors[0].Radius) || blades != liftRotors[0].NumBlades)
                {
                    foreach (var r in liftRotors)
                    {
                        double sx = r.MountPositionFrd.X >= 0 ? 1.0 : -1.0;
                        double sy = r.MountPositionFrd.Y >= 0 ? 1.0 : -1.0;
                        r.MountPositionFrd = new Vec3d(sx * newArmX, sy * newArmY, r.MountPositionFrd.Z);
                        r.Radius = radius;
                        r.NumBlades = blades;
                    }
                    if (_workingSpec.Category == VehicleAirframeCategory.Multirotor)
                        _workingSpec.ReferenceSpanMeters = 2.0 * System.Math.Sqrt(newArmX * newArmX + newArmY * newArmY);
                }
            }

            if (_workingSpec.Category == VehicleAirframeCategory.FixedWing ||
                _workingSpec.Category == VehicleAirframeCategory.HybridQuadPlaneVtol)
            {
                _workingSpec.DesignCruiseSpeedMps = System.Math.Max(5.0, EditorGUILayout.FloatField("Design Cruise Speed [m/s]", (float)_workingSpec.DesignCruiseSpeedMps));

                // Main Wing Shape & Position
                if (_workingSpec.Wings.Count > 0 && !_workingSpec.Wings[0].IsVerticalFin)
                {
                    var mainW = _workingSpec.Wings[0];
                    EditorGUILayout.Space(2);
                    GUILayout.Label("Main Wing Geometry", EditorStyles.miniBoldLabel);

                    float wingX = EditorGUILayout.Slider("Main Wing X Pos [m]", (float)mainW.ApexPositionFrd.X, -0.50f, 0.60f);
                    mainW.ApexPositionFrd = new Vec3d(wingX, mainW.ApexPositionFrd.Y, mainW.ApexPositionFrd.Z);

                    float span = EditorGUILayout.Slider("Wing Span b [m]", (float)mainW.Span, 0.50f, 6.00f);
                    float rootC = EditorGUILayout.Slider("Root Chord [m]", (float)mainW.RootChord, 0.08f, 0.90f);
                    float taper = EditorGUILayout.Slider("Taper Ratio (λ = Tip/Root)", (float)mainW.TaperRatio, 0.25f, 1.00f);
                    float sweepDeg = EditorGUILayout.Slider("Sweep Angle [deg]", (float)mainW.SweepQuarterChordDeg, -15f, 35f);
                    float dihedralDeg = EditorGUILayout.Slider("Dihedral Angle [deg]", (float)mainW.DihedralDeg, -5f, 15f);

                    mainW.Span = span;
                    mainW.RootChord = rootC;
                    mainW.TipChord = System.Math.Max(0.03, rootC * taper);
                    mainW.SweepQuarterChordDeg = sweepDeg;
                    mainW.DihedralDeg = dihedralDeg;

                    _workingSpec.ReferenceSpanMeters = mainW.Span;
                    _workingSpec.ReferenceChordMeters = mainW.MeanAerodynamicChord;
                    _workingSpec.ReferenceAreaSqM = mainW.PlanformArea;

                    EditorGUILayout.LabelField("Wing Area / MAC",
                        $"S_ref={_workingSpec.ReferenceAreaSqM:F3} m² | MAC={_workingSpec.ReferenceChordMeters:F3} m | AR={mainW.AspectRatio:F1}");
                }

                // Tailplanes (Horizontal Tail & Vertical Fin)
                WingSurfaceSpec? hTail = null;
                WingSurfaceSpec? vFin = null;
                for (int i = 1; i < _workingSpec.Wings.Count; i++)
                {
                    if (_workingSpec.Wings[i].IsVerticalFin && vFin == null) vFin = _workingSpec.Wings[i];
                    else if (!_workingSpec.Wings[i].IsVerticalFin && hTail == null) hTail = _workingSpec.Wings[i];
                }
                if (hTail != null || vFin != null)
                {
                    EditorGUILayout.Space(2);
                    GUILayout.Label("Tailplane Geometry & Position", EditorStyles.miniBoldLabel);
                    float tailRange = Mathf.Max(1.5f, (float)_workingSpec.Fuselage.Length * 0.85f);
                    if (hTail != null)
                    {
                        float hTailX = EditorGUILayout.Slider("H-Tail X Position [m]", (float)hTail.ApexPositionFrd.X, -tailRange, -0.15f);
                        hTail.ApexPositionFrd = new Vec3d(hTailX, hTail.ApexPositionFrd.Y, hTail.ApexPositionFrd.Z);
                        hTail.Span = EditorGUILayout.Slider("H-Tail Span [m]", (float)hTail.Span, 0.20f, 2.00f);
                        float hTaper = (float)hTail.TaperRatio;
                        hTail.RootChord = EditorGUILayout.Slider("H-Tail Root Chord [m]", (float)hTail.RootChord, 0.05f, 0.45f);
                        hTail.TipChord = System.Math.Max(0.03, hTail.RootChord * hTaper);
                    }
                    if (vFin != null)
                    {
                        float vFinX = EditorGUILayout.Slider("V-Fin X Position [m]", (float)vFin.ApexPositionFrd.X, -tailRange, -0.15f);
                        vFin.ApexPositionFrd = new Vec3d(vFinX, vFin.ApexPositionFrd.Y, vFin.ApexPositionFrd.Z);
                        vFin.Span = EditorGUILayout.Slider("V-Fin Height [m]", (float)vFin.Span, 0.10f, 0.80f);
                        float vTaper = (float)vFin.TaperRatio;
                        vFin.RootChord = EditorGUILayout.Slider("V-Fin Root Chord [m]", (float)vFin.RootChord, 0.06f, 0.50f);
                        vFin.TipChord = System.Math.Max(0.03, vFin.RootChord * vTaper);
                    }
                }

                // Cruise Propeller (Tractor / Pusher)
                RotorSpec? cruiseProp = null;
                foreach (var r in _workingSpec.Rotors)
                {
                    if (System.Math.Abs(r.ThrustAxisFrd.X) > System.Math.Abs(r.ThrustAxisFrd.Z))
                    {
                        cruiseProp = r;
                        break;
                    }
                }
                if (cruiseProp != null)
                {
                    EditorGUILayout.Space(2);
                    GUILayout.Label("Cruise Propeller Position & Size", EditorStyles.miniBoldLabel);
                    float propRange = Mathf.Max(1.2f, (float)_workingSpec.Fuselage.Length * 0.85f);
                    float propX = EditorGUILayout.Slider("Cruise Prop X Pos [m]", (float)cruiseProp.MountPositionFrd.X, -propRange, propRange);
                    cruiseProp.MountPositionFrd = new Vec3d(propX, cruiseProp.MountPositionFrd.Y, cruiseProp.MountPositionFrd.Z);
                    cruiseProp.Radius = EditorGUILayout.Slider("Cruise Prop Radius [m]", (float)cruiseProp.Radius, 0.06f, 0.45f);
                }
            }
            if (EditorGUI.EndChangeCheck()) _metricsDirty = true;
            EditorGUILayout.EndVertical();

            EditorGUILayout.Space(4);

            // 3. Weight & Inertia
            EditorGUILayout.BeginVertical(EditorStyles.helpBox);
            GUILayout.Label("3. Weight & Mass Properties", EditorStyles.boldLabel);
            EditorGUI.BeginChangeCheck();
            _workingSpec.MassKg = System.Math.Max(0.2, EditorGUILayout.FloatField("Total Takeoff Mass [kg]", (float)_workingSpec.MassKg));
            double cgX = EditorGUILayout.Slider("CG X Offset [m]", (float)_workingSpec.CenterOfGravityFrd.X, -0.35f, 0.35f);
            _workingSpec.CenterOfGravityFrd = new Vec3d(cgX, _workingSpec.CenterOfGravityFrd.Y, _workingSpec.CenterOfGravityFrd.Z);
            if (EditorGUI.EndChangeCheck()) _metricsDirty = true;

            EditorGUILayout.BeginHorizontal();
            _workingSpec.Ixx = System.Math.Max(0.001, EditorGUILayout.FloatField("Ixx", (float)_workingSpec.Ixx));
            _workingSpec.Iyy = System.Math.Max(0.001, EditorGUILayout.FloatField("Iyy", (float)_workingSpec.Iyy));
            _workingSpec.Izz = System.Math.Max(0.001, EditorGUILayout.FloatField("Izz", (float)_workingSpec.Izz));
            EditorGUILayout.EndHorizontal();
            if (GUILayout.Button("Auto-Estimate Inertia from Dimensions & Mass", GUILayout.Height(22)))
                EstimateInertiaFromDimensions();
            EditorGUILayout.EndVertical();

            EditorGUILayout.Space(4);

            // 4. Motor Power & Propulsion
            EditorGUILayout.BeginVertical(EditorStyles.helpBox);
            GUILayout.Label($"4. Motor Power & Propulsion ({_workingSpec.Rotors.Count} Motors)", EditorStyles.boldLabel);
            if (_workingSpec.Rotors.Count > 0)
            {
                var r0 = _workingSpec.Rotors[0];
                EditorGUI.BeginChangeCheck();
                float maxPowerW = EditorGUILayout.FloatField("Motor Max Power [W/motor]", (float)r0.MaxMotorPowerWatts);
                float kv = EditorGUILayout.FloatField("Motor KV [RPM/V]", (float)r0.MotorKv);
                float maxRpm = EditorGUILayout.Slider("Max RPM", (float)r0.MaxRpm, 3000f, 25000f);
                float idleRpm = EditorGUILayout.Slider("Idle RPM", (float)r0.IdleRpm, 300f, maxRpm * 0.5f);
                if (EditorGUI.EndChangeCheck())
                {
                    foreach (var r in _workingSpec.Rotors)
                    {
                        r.MaxMotorPowerWatts = System.Math.Max(20.0, maxPowerW);
                        r.MotorKv = System.Math.Max(100.0, kv);
                        r.MaxRpm = maxRpm;
                        r.IdleRpm = idleRpm;
                    }
                    _metricsDirty = true;
                }
                double weightN = _workingSpec.MassKg * 9.80665;
                double twRatio = weightN > 1e-6 ? _cachedMaxThrustN / weightN : 0.0;
                double totalRatedW = r0.MaxMotorPowerWatts * _workingSpec.Rotors.Count;
                EditorGUILayout.HelpBox(
                    $"Rated Total Power: {totalRatedW:F0} W | BEMT Max Thrust: {_cachedMaxThrustN:F1} N (T/W={twRatio:F2}) | Max Shaft Power: {_cachedShaftPowerW:F0} W",
                    MessageType.None);
            }
            EditorGUILayout.EndVertical();

            EditorGUILayout.Space(4);

            // 5. Battery & Spec Notes
            EditorGUILayout.BeginVertical(EditorStyles.helpBox);
            GUILayout.Label("5. Battery & Engineering Spec Notes", EditorStyles.boldLabel);
            _workingSpec.BatteryCells = EditorGUILayout.IntSlider("Battery Cells (S)", Mathf.RoundToInt((float)_workingSpec.BatteryCells), 2, 14);
            _workingSpec.BatteryCapacityMah = System.Math.Max(500.0, EditorGUILayout.FloatField("Battery Capacity [mAh]", (float)_workingSpec.BatteryCapacityMah));
            _workingSpec.BatteryInternalResistanceOhm = System.Math.Max(0.002, EditorGUILayout.FloatField("Internal Resistance [Ω]", (float)_workingSpec.BatteryInternalResistanceOhm));

            double nominalV = _workingSpec.BatteryCells * 3.7;
            double energyWh = nominalV * (_workingSpec.BatteryCapacityMah / 1000.0);
            double estCruisePowerW = System.Math.Max(40.0, _workingSpec.MassKg * 95.0);
            double estFlightMin = (energyWh * 0.80 / estCruisePowerW) * 60.0;
            EditorGUILayout.LabelField("Battery Summary", $"{_workingSpec.BatteryCells:F0}S ({nominalV:F1} V) | {energyWh:F1} Wh | Est. Endurance ~{estFlightMin:F1} min");

            GUILayout.Label("Spec Notes (Motor / ESC / Battery / Payload):", EditorStyles.miniLabel);
            _workingSpec.SpecNotes = EditorGUILayout.TextArea(_workingSpec.SpecNotes ?? "", GUILayout.MinHeight(48f));
            EditorGUILayout.EndVertical();

            if (_workingSpec.Category != VehicleAirframeCategory.Multirotor)
            {
                EditorGUILayout.Space(4);

                // 6. Aerodynamic Solvers (BEMT, 2D Airfoil Polar) & 3D Mesh Geometry Extraction (Fixed-Wing / VTOL)
                EditorGUILayout.BeginVertical(EditorStyles.helpBox);
                _showAeroSolvers = EditorGUILayout.Foldout(_showAeroSolvers, "6. Aero Analysis Tools (BEMT Rotor & 2D Airfoil Solvers)", true, EditorStyles.foldoutHeader);
                if (_showAeroSolvers)
                {
                    if (_host != null && _host.SceneVehicle != null)
                    {
                        if (GUILayout.Button("Extract Mass/Inertia from 3D Meshes", GUILayout.Height(24f)))
                        {
                            Undo.RecordObject(_host.SceneVehicle, "Extract 3D Mesh Geometry");
                            _host.SceneVehicle.ExtractGeometryFromChildMeshesAndBake();
                            EditorUtility.SetDirty(_host.SceneVehicle);
                            var updated = _host.SceneVehicle.ActiveConfig;
                            _workingSpec.MassKg = updated.MassKg;
                            _workingSpec.Ixx = updated.Ixx;
                            _workingSpec.Iyy = updated.Iyy;
                            _workingSpec.Izz = updated.Izz;
                            _metricsDirty = true;
                        }
                        EditorGUILayout.Space(4);
                    }

                    GUILayout.Label("Module 4: CCBlade BEMT Rotor Thrust / Efficiency / Ground Effect (IGE)", EditorStyles.miniBoldLabel);
                    _testRotorRpm = EditorGUILayout.Slider("Rotor Speed [RPM]", _testRotorRpm, 500f, 15000f);
                    _testRotorAxialSpeedMps = EditorGUILayout.Slider("Axial Inflow V_inf [m/s]", _testRotorAxialSpeedMps, -5f, 35f);
                    _testRotorAglMeters = EditorGUILayout.Slider("Altitude AGL [m]", _testRotorAglMeters, 0.1f, 20f);
                    if (GUILayout.Button("Compute Rotor BEMT Performance (incl. Ground Effect)"))
                    {
                        var atmDesign = IsaAtmosphereAndTurbulence.Evaluate(_workingSpec.DesignAltitudeMsl);
                        RotorSpec rSpec = _workingSpec.Rotors.Count > 0 ? _workingSpec.Rotors[0] : AircraftConfigSpec.CreateStandardQuadcopterX().Rotors[0];
                        _lastRotorSolution = CcBladeBemtSolver.EvaluateRotor(rSpec, rSpec.ThrustAxisFrd.Normalized() * _testRotorAxialSpeedMps, Vec3d.Zero, _testRotorRpm, atmDesign, _testRotorAglMeters, true, true);
                        _hasRotorResult = true;
                    }
                    if (_hasRotorResult)
                    {
                        EditorGUILayout.HelpBox(
                            $"Single Rotor Thrust={_lastRotorSolution.ThrustNewtons:F2} N (Total {_workingSpec.Rotors.Count}x={_lastRotorSolution.ThrustNewtons * _workingSpec.Rotors.Count:F1} N) | Torque={_lastRotorSolution.AerodynamicTorqueNm:F4} Nm | Shaft Power={_lastRotorSolution.ShaftPowerWatts:F1} W | Eta={(_lastRotorSolution.EfficiencyEta * 100.0):F1}%",
                            MessageType.Info);
                    }

                    EditorGUILayout.Space(4);
                    GUILayout.Label("Module 1: 2D Airfoil Viscous Polar & Viterna (±180°) Solver", EditorStyles.miniBoldLabel);
                    _nacaCode = EditorGUILayout.TextField("NACA 4-Digit Code", _nacaCode);
                    _testAlphaDeg = EditorGUILayout.Slider("Angle of Attack Alpha [deg]", _testAlphaDeg, -180f, 180f);
                    _testReynolds = EditorGUILayout.Slider("Reynolds Number (Re)", _testReynolds, 1.0e4f, 1.0e7f);
                    _testMach = EditorGUILayout.Slider("Mach Number (M < 0.7)", _testMach, 0.0f, 0.68f);
                    if (GUILayout.Button("Solve 2D Airfoil Polar"))
                    {
                        var af = AirfoilSpec.FromNaca4Digit(_nacaCode, _testReynolds);
                        _lastAirfoilPoint = AirfoilPolarSolver.Evaluate(af, _testAlphaDeg * FrameConverter.Deg2Rad, _testReynolds, _testMach);
                        _hasAirfoilResult = true;
                    }
                    if (_hasAirfoilResult)
                    {
                        EditorGUILayout.HelpBox(
                            $"Alpha={_lastAirfoilPoint.AlphaDeg:F1} deg | Cl={_lastAirfoilPoint.Cl:F4} | Cd={_lastAirfoilPoint.Cd:F4} | Cm={_lastAirfoilPoint.Cm:F4} | L/D={(_lastAirfoilPoint.Cl / System.Math.Max(1e-4, _lastAirfoilPoint.Cd)):F1}",
                            MessageType.Info);
                    }
                }
                EditorGUILayout.EndVertical();
            }

            EditorGUILayout.EndScrollView();

            // Footer Apply / Reset / Close
            EditorGUILayout.Space(4);
            EditorGUILayout.BeginHorizontal();
            GUI.backgroundColor = new Color(0.55f, 0.95f, 0.65f);
            if (GUILayout.Button("Apply, Bake Aero & Close", GUILayout.Height(30f)))
            {
                ApplyToHost();
                Close();
            }
            GUI.backgroundColor = Color.white;
            if (GUILayout.Button("Reset to Preset", GUILayout.Width(120f), GUILayout.Height(30f)))
            {
                LoadPreset(_workingSpec.Category);
            }
            if (GUILayout.Button("Close", GUILayout.Width(80f), GUILayout.Height(30f)))
            {
                Close();
            }
            EditorGUILayout.EndHorizontal();
        }

        private void DrawCategoryButton(string label, VehicleAirframeCategory category)
        {
            GUI.backgroundColor = _workingSpec.Category == category ? new Color(0.65f, 1f, 0.7f) : Color.white;
            if (GUILayout.Button(label, GUILayout.Height(26f)) && _workingSpec.Category != category)
            {
                LoadPreset(category);
            }
            GUI.backgroundColor = Color.white;
        }

        private void LoadPreset(VehicleAirframeCategory category)
        {
            _workingSpec = category switch
            {
                VehicleAirframeCategory.FixedWing => AircraftConfigSpec.CreateFixedWingUav(),
                VehicleAirframeCategory.HybridQuadPlaneVtol => AircraftConfigSpec.CreateQuadPlaneVtolMillie(),
                _ => AircraftConfigSpec.CreateStandardQuadcopterX()
            };
            _metricsDirty = true;
        }

        private void ApplyToHost()
        {
            if (_host == null) return;
            string model = _workingSpec.Category switch
            {
                VehicleAirframeCategory.FixedWing => "plane",
                VehicleAirframeCategory.HybridQuadPlaneVtol => "standard_vtol",
                _ => "iris"
            };
            if (_host.GetCurrentConfig().Category != _workingSpec.Category)
            {
                _host.SwitchAirframeEverywhere(_workingSpec.Category, model);
            }
            var cloned = JsonUtility.FromJson<AircraftConfigSpec>(JsonUtility.ToJson(_workingSpec));
            if (cloned != null)
            {
                _host.RebakeActiveConfig(cloned);
                _host.Repaint();
            }
        }

        private void EstimateInertiaFromDimensions()
        {
            double m = _workingSpec.MassKg;
            double b = System.Math.Max(0.3, _workingSpec.ReferenceSpanMeters);
            double l = System.Math.Max(0.25, _workingSpec.Fuselage.Length);
            double h = System.Math.Max(0.10, _workingSpec.Fuselage.MaxDiameter);
            _workingSpec.Ixx = m * (b * b * 0.08 + h * h * 0.08);
            _workingSpec.Iyy = m * (l * l * 0.08 + h * h * 0.08);
            _workingSpec.Izz = m * (b * b * 0.08 + l * l * 0.08);
        }

        private void RecomputePropulsionMetrics()
        {
            var atm = IsaAtmosphereAndTurbulence.Evaluate(_workingSpec.DesignAltitudeMsl);
            double thrust = 0.0;
            double power = 0.0;
            foreach (var r in _workingSpec.Rotors)
            {
                var sol = CcBladeBemtSolver.EvaluateRotor(r, Vec3d.Zero, Vec3d.Zero, r.MaxRpm, atm, 10.0, false, false);
                thrust += sol.ThrustNewtons;
                power += sol.ShaftPowerWatts;
            }
            _cachedMaxThrustN = thrust;
            _cachedShaftPowerW = power;
        }

        private List<RotorSpec> GetVerticalLiftRotors()
        {
            var list = new List<RotorSpec>();
            foreach (var r in _workingSpec.Rotors)
            {
                if (System.Math.Abs(r.ThrustAxisFrd.Z) >= System.Math.Abs(r.ThrustAxisFrd.X))
                    list.Add(r);
            }
            return list;
        }

        private static float EstimateMultirotorWheelbaseMeters(AircraftConfigSpec spec)
        {
            float maxDist = 0.45f;
            foreach (var r in spec.Rotors)
            {
                float d = 2f * Mathf.Sqrt((float)(r.MountPositionFrd.X * r.MountPositionFrd.X + r.MountPositionFrd.Y * r.MountPositionFrd.Y));
                if (d > maxDist) maxDist = d;
            }
            return maxDist;
        }

        private static float ComputeBlueprintExtentMeters(AircraftConfigSpec spec)
        {
            float maxE = Mathf.Max(0.5f, (float)spec.Fuselage.Length * 0.55f);
            maxE = Mathf.Max(maxE, (float)spec.ReferenceSpanMeters * 0.55f);
            foreach (var r in spec.Rotors)
            {
                maxE = Mathf.Max(maxE, Mathf.Abs((float)r.MountPositionFrd.X) + (float)r.Radius);
                maxE = Mathf.Max(maxE, Mathf.Abs((float)r.MountPositionFrd.Y) + (float)r.Radius);
            }
            return maxE;
        }

        private static void DrawGrid(Rect rect, float spacing, Color color)
        {
            Handles.BeginGUI();
            Handles.color = color;
            for (float x = rect.x; x < rect.xMax; x += spacing)
                Handles.DrawLine(new Vector2(x, rect.y), new Vector2(x, rect.yMax));
            for (float y = rect.y; y < rect.yMax; y += spacing)
                Handles.DrawLine(new Vector2(rect.x, y), new Vector2(rect.xMax, y));
            Handles.EndGUI();
        }
    }
}
