// Licensed under the Apache License, Version 2.0.
// Algorithmic references: AeroSandbox 3D VLM & MachUpX Numerical Lifting-Line (MIT License)
using System;
using System.Collections.Generic;
using FlightDynamics6DoF.Core.Math;
using FlightDynamics6DoF.Core.Specs;

namespace FlightDynamics6DoF.Core.Aerodynamics
{
    public struct WingPanelElement
    {
        public int WingIndex;
        public bool IsVerticalFin;
        public bool IsRightHalf;
        public Vec3d VortexBoundLeftFrd;
        public Vec3d VortexBoundRightFrd;
        public Vec3d QuarterChordCenterFrd;
        public Vec3d CollocationThreeQuarterFrd;
        public Vec3d NormalFrd;
        public Vec3d SpanwiseUnitFrd;
        public Vec3d ChordwiseUnitFrd;
        public double Chord;
        public double PanelWidth;
        public double PanelArea;
        public double LocalIncidenceRad;
        public double FlapFraction;
        public bool HasControlSurface;
        public ControlSurfaceType ControlType;
        public int ControlChannel;
        public double MaxDeflectionRad;
    }

    public struct LiftingSurfaceSystemResult
    {
        public Vec3d TotalForceFrd;
        public Vec3d TotalMomentAboutCgFrd;
        public double TotalLiftNewtons;
        public double TotalInducedDragNewtons;
        public double TotalProfileDragNewtons;
        public double[] PanelCirculations;
        public double[] PanelEffectiveAlphaDeg;
        public double[] PanelCl;
    }

    /// <summary>
    /// Module 2: 3D Multi-Lifting-Surface Vortex Lattice Method (VLM) and Nonlinear Lifting-Line Strip Solver.
    /// Computes Biot-Savart horseshoe vortex downwash/sidewash across Main Wing, Horizontal Tail, Vertical Fin, and Canard,
    /// couples with Module 1's 2D viscous polar (+/-180 deg Viterna), and superimposes Module 4's rotor slipstream wake.
    /// </summary>
    public static class VlmAndLiftingLineSolver
    {
        public static List<WingPanelElement> BuildPanels(IList<WingSurfaceSpec> wings)
        {
            var panels = new List<WingPanelElement>();
            if (wings == null) return panels;

            for (int wIdx = 0; wIdx < wings.Count; wIdx++)
            {
                var w = wings[wIdx];
                int nPanels = System.Math.Max(2, w.SpanwisePanels);
                if (!w.IsVerticalFin && nPanels % 2 != 0)
                    nPanels++;

                double sweepRad = w.SweepQuarterChordDeg * FrameConverter.Deg2Rad;
                double dihedralRad = w.DihedralDeg * FrameConverter.Deg2Rad;

                if (w.IsVerticalFin)
                {
                    // Vertical fin extends upward along -Z_FRD from ApexPositionFrd
                    double dz = w.Span / nPanels;
                    for (int i = 0; i < nPanels; i++)
                    {
                        double eta0 = (double)i / nPanels;
                        double eta1 = (double)(i + 1) / nPanels;
                        double etaMid = 0.5 * (eta0 + eta1);

                        double chordMid = w.RootChord + (w.TipChord - w.RootChord) * etaMid;
                        double xOffset0 = (eta0 * w.Span) * System.Math.Tan(sweepRad);
                        double xOffset1 = (eta1 * w.Span) * System.Math.Tan(sweepRad);
                        double xOffsetMid = 0.5 * (xOffset0 + xOffset1);

                        Vec3d pLeft = w.ApexPositionFrd + new Vec3d(-xOffset0 - 0.25 * w.RootChord, 0.0, -eta0 * w.Span);
                        Vec3d pRight = w.ApexPositionFrd + new Vec3d(-xOffset1 - 0.25 * w.TipChord, 0.0, -eta1 * w.Span);
                        Vec3d qcCenter = 0.5 * (pLeft + pRight);
                        Vec3d colloc = qcCenter + new Vec3d(-0.5 * chordMid, 0.0, 0.0);

                        bool hasFlap = w.ControlType != ControlSurfaceType.None &&
                                       etaMid >= w.FlapSpanStartRatio && etaMid <= w.FlapSpanEndRatio;

                        panels.Add(new WingPanelElement
                        {
                            WingIndex = wIdx,
                            IsVerticalFin = true,
                            IsRightHalf = true,
                            VortexBoundLeftFrd = pLeft,
                            VortexBoundRightFrd = pRight,
                            QuarterChordCenterFrd = qcCenter,
                            CollocationThreeQuarterFrd = colloc,
                            NormalFrd = new Vec3d(0.0, 1.0, 0.0), // Side force normal +Y_FRD
                            SpanwiseUnitFrd = new Vec3d(0.0, 0.0, -1.0),
                            ChordwiseUnitFrd = new Vec3d(-1.0, 0.0, 0.0),
                            Chord = chordMid,
                            PanelWidth = dz,
                            PanelArea = chordMid * dz,
                            LocalIncidenceRad = w.RootIncidenceDeg * FrameConverter.Deg2Rad,
                            FlapFraction = hasFlap ? w.FlapChordRatio : 0.0,
                            HasControlSurface = hasFlap,
                            ControlType = w.ControlType,
                            ControlChannel = w.ActuatorChannelIndex,
                            MaxDeflectionRad = w.MaxDeflectionDeg * FrameConverter.Deg2Rad
                        });
                    }
                }
                else
                {
                    int halfN = nPanels / 2;
                    double halfSpan = 0.5 * w.Span;

                    // Left (-Y) to Right (+Y) panels
                    for (int i = -halfN; i < halfN; i++)
                    {
                        bool isRight = i >= 0;
                        double y0 = (double)i / halfN * halfSpan;
                        double y1 = (double)(i + 1) / halfN * halfSpan;
                        double yMid = 0.5 * (y0 + y1);

                        double eta0 = System.Math.Abs(y0) / halfSpan;
                        double eta1 = System.Math.Abs(y1) / halfSpan;
                        double etaMid = System.Math.Abs(yMid) / halfSpan;

                        double c0 = w.RootChord + (w.TipChord - w.RootChord) * eta0;
                        double c1 = w.RootChord + (w.TipChord - w.RootChord) * eta1;
                        double cMid = 0.5 * (c0 + c1);

                        double xQc0 = w.ApexPositionFrd.X - 0.25 * w.RootChord - System.Math.Abs(y0) * System.Math.Tan(sweepRad);
                        double xQc1 = w.ApexPositionFrd.X - 0.25 * w.RootChord - System.Math.Abs(y1) * System.Math.Tan(sweepRad);

                        double z0 = w.ApexPositionFrd.Z - System.Math.Abs(y0) * System.Math.Tan(dihedralRad);
                        double z1 = w.ApexPositionFrd.Z - System.Math.Abs(y1) * System.Math.Tan(dihedralRad);

                        Vec3d pLeft = new Vec3d(xQc0, y0, z0);
                        Vec3d pRight = new Vec3d(xQc1, y1, z1);
                        Vec3d qcCenter = 0.5 * (pLeft + pRight);
                        Vec3d colloc = qcCenter + new Vec3d(-0.5 * cMid, 0.0, 0.0);

                        double localIncidenceDeg = w.RootIncidenceDeg + w.TipWashoutDeg * etaMid;
                        double signDihedral = isRight ? 1.0 : -1.0;
                        Vec3d normalFrd = new Vec3d(
                            0.0,
                            signDihedral * System.Math.Sin(dihedralRad),
                            -System.Math.Cos(dihedralRad)).Normalized();

                        bool hasFlap = w.ControlType != ControlSurfaceType.None &&
                                       etaMid >= w.FlapSpanStartRatio && etaMid <= w.FlapSpanEndRatio;

                        double dy = System.Math.Abs(y1 - y0) / System.Math.Max(0.2, System.Math.Cos(dihedralRad));

                        panels.Add(new WingPanelElement
                        {
                            WingIndex = wIdx,
                            IsVerticalFin = false,
                            IsRightHalf = isRight,
                            VortexBoundLeftFrd = pLeft,
                            VortexBoundRightFrd = pRight,
                            QuarterChordCenterFrd = qcCenter,
                            CollocationThreeQuarterFrd = colloc,
                            NormalFrd = normalFrd,
                            SpanwiseUnitFrd = (pRight - pLeft).Normalized(),
                            ChordwiseUnitFrd = new Vec3d(-1.0, 0.0, 0.0),
                            Chord = cMid,
                            PanelWidth = dy,
                            PanelArea = cMid * dy,
                            LocalIncidenceRad = localIncidenceDeg * FrameConverter.Deg2Rad,
                            FlapFraction = hasFlap ? w.FlapChordRatio : 0.0,
                            HasControlSurface = hasFlap,
                            ControlType = w.ControlType,
                            ControlChannel = w.ActuatorChannelIndex,
                            MaxDeflectionRad = w.MaxDeflectionDeg * FrameConverter.Deg2Rad
                        });
                    }
                }
            }

            return panels;
        }

        public static LiftingSurfaceSystemResult Evaluate(
            IList<WingSurfaceSpec> wings,
            IList<WingPanelElement> panels,
            Vec3d aircraftVelocityFrd,
            Vec3d aircraftAngularVelocityFrd,
            Vec3d cgFrd,
            AtmosphereState atm,
            double[]? actuatorControlsNormalized,
            IList<RotorSpec>? rotors = null,
            RotorBemtSolution[]? rotorSolutions = null,
            bool enable3DVlmWakeInduction = true)
        {
            int n = panels.Count;
            if (n == 0 || wings == null || wings.Count == 0)
            {
                return new LiftingSurfaceSystemResult
                {
                    PanelCirculations = Array.Empty<double>(),
                    PanelEffectiveAlphaDeg = Array.Empty<double>(),
                    PanelCl = Array.Empty<double>()
                };
            }

            double vFreestreamMag = aircraftVelocityFrd.Magnitude;
            Vec3d freestreamWakeDir = vFreestreamMag > 0.5
                ? (-aircraftVelocityFrd).Normalized()
                : new Vec3d(-1.0, 0.0, 0.0);

            Vec3d[] localVelAtQc = new Vec3d[n];
            double[] controlDeflectionRad = new double[n];

            for (int i = 0; i < n; i++)
            {
                var p = panels[i];
                Vec3d rArm = p.QuarterChordCenterFrd - cgFrd;
                Vec3d vLocal = aircraftVelocityFrd + Vec3d.Cross(aircraftAngularVelocityFrd, rArm);

                // Superimpose Rotor Slipstream Tube wake velocity (Module 4 -> Module 2 coupling)
                if (rotors != null && rotorSolutions != null)
                {
                    int rCount = System.Math.Min(rotors.Count, rotorSolutions.Length);
                    for (int rIdx = 0; rIdx < rCount; rIdx++)
                    {
                        Vec3d vSlip = CcBladeBemtSolver.ComputeSlipstreamInducedVelocityFrd(
                            rotors[rIdx], rotorSolutions[rIdx], p.QuarterChordCenterFrd);
                        // Recall aircraftVelocityFrd is the velocity of the aircraft through the air;
                        // slipstream moving backward (-X) increases effective oncoming airflow (+X) over the wing.
                        vLocal -= vSlip;
                    }
                }

                localVelAtQc[i] = vLocal;
                controlDeflectionRad[i] = GetPanelControlDeflectionRad(p, actuatorControlsNormalized);
            }

            // Optional 3D VLM Horseshoe Vortex Induced Velocity calculation
            Vec3d[] vortexInducedVel = new Vec3d[n];
            double[] circulations = new double[n];

            if (enable3DVlmWakeInduction && vFreestreamMag > 1.0)
            {
                double[,] aic = new double[n, n];
                double[] rhs = new double[n];

                for (int i = 0; i < n; i++)
                {
                    var pi = panels[i];
                    var wingSpec = wings[pi.WingIndex];
                    Vec3d vColloc = localVelAtQc[i] + Vec3d.Cross(
                        aircraftAngularVelocityFrd,
                        pi.CollocationThreeQuarterFrd - pi.QuarterChordCenterFrd);

                    double flapEff = AirfoilPolarSolver.ComputeFlapEffectiveness(pi.FlapFraction);
                    double alpha0 = -2.05 * wingSpec.Airfoil.MaxCamberRatio - flapEff * controlDeflectionRad[i];
                    double geometricTilt = pi.LocalIncidenceRad - alpha0;

                    // Normal component of freestream + camber/incidence boundary condition (aic[i,i] < 0 for NormalFrd = -Z)
                    rhs[i] = Vec3d.Dot(vColloc, pi.NormalFrd) - vColloc.Magnitude * System.Math.Sin(geometricTilt);

                    for (int j = 0; j < n; j++)
                    {
                        Vec3d uInd = ComputeHorseshoeVortexVelocity(
                            panels[j].VortexBoundLeftFrd,
                            panels[j].VortexBoundRightFrd,
                            freestreamWakeDir,
                            pi.CollocationThreeQuarterFrd,
                            includeBoundSegment: true);
                        aic[i, j] = Vec3d.Dot(uInd, pi.NormalFrd);
                    }
                }

                circulations = LinearAlgebra.SolveLinearSystem(aic, rhs);

                // Compute trailing-wake downwash/sidewash at each panel's quarter-chord
                for (int i = 0; i < n; i++)
                {
                    Vec3d wInd = Vec3d.Zero;
                    for (int j = 0; j < n; j++)
                    {
                        Vec3d uTrail = ComputeHorseshoeVortexVelocity(
                            panels[j].VortexBoundLeftFrd,
                            panels[j].VortexBoundRightFrd,
                            freestreamWakeDir,
                            panels[i].QuarterChordCenterFrd,
                            includeBoundSegment: false);
                        wInd += uTrail * circulations[j];
                    }
                    vortexInducedVel[i] = wInd;
                }
            }

            Vec3d totalForceFrd = Vec3d.Zero;
            Vec3d totalMomentCgFrd = Vec3d.Zero;
            double totalLift = 0.0;
            double totalInducedDrag = 0.0;
            double totalProfileDrag = 0.0;
            double[] panelAlphaDeg = new double[n];
            double[] panelCl = new double[n];

            for (int i = 0; i < n; i++)
            {
                var p = panels[i];
                var wSpec = wings[p.WingIndex];

                // Effective airflow velocity vector at panel
                Vec3d vEff = localVelAtQc[i] - vortexInducedVel[i];
                double vMag = vEff.Magnitude;
                if (vMag < 0.05)
                    continue;

                double uForward = vEff.X;
                double vNormal = -Vec3d.Dot(vEff, p.NormalFrd);
                double alphaLocal = System.Math.Atan2(vNormal, System.Math.Max(0.05, uForward)) + p.LocalIncidenceRad;

                // Finite-wing Prandtl lifting-line downwash approximation when running fast Strip mode (Level 2)
                double ar = wSpec.AspectRatio;
                if (!enable3DVlmWakeInduction)
                {
                    double eOswald = 1.78 * (1.0 - 0.045 * System.Math.Pow(ar, 0.68)) - 0.64;
                    eOswald = System.Math.Clamp(eOswald, 0.65, 0.92);
                    alphaLocal /= (1.0 + 2.0 / (ar * eOswald));
                }

                double re = IsaAtmosphereAndTurbulence.ComputeReynolds(vMag, p.Chord, atm);
                double mach = IsaAtmosphereAndTurbulence.ComputeMach(vMag, atm);

                var polar = AirfoilPolarSolver.Evaluate(
                    wSpec.Airfoil,
                    alphaLocal,
                    re,
                    mach,
                    ar,
                    controlDeflectionRad[i],
                    p.FlapFraction);

                panelAlphaDeg[i] = alphaLocal * FrameConverter.Rad2Deg;
                panelCl[i] = polar.Cl;

                double qDyn = 0.5 * atm.DensityKgPerM3 * vMag * vMag;
                double dLift = qDyn * p.PanelArea * polar.Cl;

                // Induced drag from VLM downwash angle or Prandtl AR formula
                double alphaInduced = enable3DVlmWakeInduction
                    ? System.Math.Atan2(-Vec3d.Dot(vortexInducedVel[i], p.NormalFrd), System.Math.Max(0.5, vMag))
                    : (polar.Cl / (System.Math.PI * System.Math.Max(2.0, ar) * 0.82));
                double cdInduced = System.Math.Max(0.0, polar.Cl * System.Math.Sin(System.Math.Abs(alphaInduced)));

                double dDragInduced = qDyn * p.PanelArea * cdInduced;
                double dDragProfile = qDyn * p.PanelArea * polar.Cd;
                double dDragTotal = dDragInduced + dDragProfile;
                double dMomentPitch = qDyn * p.PanelArea * p.Chord * polar.Cm;

                Vec3d windUnit = vEff.Normalized();
                Vec3d dragDir = -windUnit;
                Vec3d liftDir = Vec3d.Cross(windUnit, p.SpanwiseUnitFrd).Normalized();
                if (Vec3d.Dot(liftDir, p.NormalFrd) < 0.0)
                    liftDir = -liftDir;

                Vec3d dForce = liftDir * dLift + dragDir * dDragTotal;
                Vec3d rArm = p.QuarterChordCenterFrd - cgFrd;
                Vec3d dMoment = Vec3d.Cross(rArm, dForce) + p.SpanwiseUnitFrd * dMomentPitch;

                totalForceFrd += dForce;
                totalMomentCgFrd += dMoment;
                totalLift += dLift;
                totalInducedDrag += dDragInduced;
                totalProfileDrag += dDragProfile;

                if (!enable3DVlmWakeInduction)
                    circulations[i] = 0.5 * vMag * p.Chord * polar.Cl;
            }

            return new LiftingSurfaceSystemResult
            {
                TotalForceFrd = totalForceFrd,
                TotalMomentAboutCgFrd = totalMomentCgFrd,
                TotalLiftNewtons = totalLift,
                TotalInducedDragNewtons = totalInducedDrag,
                TotalProfileDragNewtons = totalProfileDrag,
                PanelCirculations = circulations,
                PanelEffectiveAlphaDeg = panelAlphaDeg,
                PanelCl = panelCl
            };
        }

        private static double GetPanelControlDeflectionRad(WingPanelElement p, double[]? controls)
        {
            if (!p.HasControlSurface || controls == null || p.ControlChannel < 0 || p.ControlChannel >= controls.Length)
                return 0.0;

            double cmd = System.Math.Clamp(controls[p.ControlChannel], -1.0, 1.0);
            switch (p.ControlType)
            {
                case ControlSurfaceType.Aileron:
                {
                    // Dual aileron support (e.g., PX4 PWM_MAIN_FUNC6/7 on Ch 5 & Ch 6):
                    // If a valid secondary aileron channel is actively driven, map left/right halves directly.
                    int secChan = p.ControlChannel + 1;
                    if (secChan < controls.Length && System.Math.Abs(controls[secChan]) > 1e-4 && p.IsRightHalf)
                    {
                        double secCmd = System.Math.Clamp(controls[secChan], -1.0, 1.0);
                        return secCmd * p.MaxDeflectionRad;
                    }

                    // Single unified roll channel (e.g. Ch 5 / roll command):
                    // Positive roll command (+cmd) deflects right aileron UP (-Cl) and left aileron DOWN (+Cl)
                    return (p.IsRightHalf ? -1.0 : 1.0) * cmd * p.MaxDeflectionRad;
                }
                case ControlSurfaceType.Elevator:
                case ControlSurfaceType.Flap:
                case ControlSurfaceType.Rudder:
                case ControlSurfaceType.Elevon:
                    return cmd * p.MaxDeflectionRad;
                default:
                    return 0.0;
            }
        }

        /// <summary>
        /// Evaluates velocity induced at point P by a unit-circulation horseshoe vortex (bound segment A->B plus semi-infinite trailing legs along wakeDir).
        /// Uses finite-core Vatistas regularization to prevent numerical singularities.
        /// </summary>
        private static Vec3d ComputeHorseshoeVortexVelocity(
            Vec3d a,
            Vec3d b,
            Vec3d wakeDir,
            Vec3d p,
            bool includeBoundSegment)
        {
            const double coreRadiusSq = 1.0e-5;

            Vec3d vTotal = Vec3d.Zero;

            if (includeBoundSegment)
            {
                Vec3d r0 = b - a;
                Vec3d r1 = p - a;
                Vec3d r2 = p - b;
                Vec3d cross = Vec3d.Cross(r1, r2);
                double crossSq = cross.MagnitudeSquared;
                double r1Mag = System.Math.Max(1e-6, r1.Magnitude);
                double r2Mag = System.Math.Max(1e-6, r2.Magnitude);

                double dotTerm = Vec3d.Dot(r0, r1 / r1Mag - r2 / r2Mag);
                vTotal += cross * (MathConstants.InvFourPi * dotTerm / (crossSq + coreRadiusSq));
            }

            // Left trailing vortex from infinity to A (so -wakeDir from A to infinity)
            vTotal -= ComputeSemiInfiniteVortexLeg(a, wakeDir, p, coreRadiusSq);
            // Right trailing vortex from B to infinity along wakeDir
            vTotal += ComputeSemiInfiniteVortexLeg(b, wakeDir, p, coreRadiusSq);

            return vTotal;
        }

        private static Vec3d ComputeSemiInfiniteVortexLeg(Vec3d origin, Vec3d dirUnit, Vec3d p, double coreSq)
        {
            Vec3d r1 = p - origin;
            double r1Mag = System.Math.Max(1e-6, r1.Magnitude);
            Vec3d cross = Vec3d.Cross(dirUnit, r1);
            double crossSq = cross.MagnitudeSquared;
            double factor = 1.0 + Vec3d.Dot(dirUnit, r1) / r1Mag;
            return cross * (MathConstants.InvFourPi * factor / (crossSq + coreSq));
        }
    }
}
