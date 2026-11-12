// Licensed under the Apache License, Version 2.0.
using System;
using System.Collections.Generic;
using FlightDynamics6DoF.Core.Gnc;
using FlightDynamics6DoF.Core.Math;
using FlightDynamics6DoF.Core.Specs;

namespace FlightDynamics6DoF.Core.Aerodynamics
{
    /// <summary>
    /// Module 5 &amp; Stage 1 Pipeline: Full-Aircraft Pre-Simulation Geometry, Aerodynamic, Stability &amp; Trim Analyzer.
    /// Executes Modules 1-4 offline/on-demand to build a complete <see cref="BakedAeroDatabase"/> that drives
    /// Stage 2 Real-Time Simulation (Level 0 ~ Level 3) and the Unity Editor Engineering Workbench.
    /// </summary>
    public static class StabilityAndTrimAnalyzer
    {
        /// <summary>
        /// Estimates aircraft mass, inertia tensor, reference areas, and bluff drag parameters from a 3D mesh bounding box
        /// when a user imports an arbitrary 3D drone or aircraft model.
        /// </summary>
        public static void EstimateGeometryFromBoundingBox(
            AircraftConfigSpec config,
            Vec3d boundingBoxSizeFrd,
            double estimatedMassKg)
        {
            double lx = System.Math.Max(0.10, boundingBoxSizeFrd.X);
            double ly = System.Math.Max(0.10, boundingBoxSizeFrd.Y);
            double lz = System.Math.Max(0.05, boundingBoxSizeFrd.Z);

            config.MassKg = System.Math.Max(0.1, estimatedMassKg);
            // Mass distribution factor (central battery/avionics concentration ~0.55 of uniform cuboid)
            const double kInertia = 0.55 / 12.0;
            config.Ixx = config.MassKg * kInertia * (ly * ly + lz * lz);
            config.Iyy = config.MassKg * kInertia * (lx * lx + lz * lz);
            config.Izz = config.MassKg * kInertia * (lx * lx + ly * ly);

            config.Fuselage.Length = lx;
            config.Fuselage.MaxDiameter = System.Math.Min(ly, lz);
            config.Fuselage.ProjectedAreasFrd = new Vec3d(
                0.45 * ly * lz,
                0.45 * lx * lz,
                0.42 * lx * ly);
            config.Fuselage.WettedArea = 2.0 * (config.Fuselage.ProjectedAreasFrd.X +
                                                config.Fuselage.ProjectedAreasFrd.Y +
                                                config.Fuselage.ProjectedAreasFrd.Z);
        }

        /// <summary>
        /// Runs the complete Stage 1 Pre-Simulation Analysis Pipeline on <paramref name="config"/>
        /// and returns the baked aerodynamic, propulsion, stability, trim, and control-allocation database.
        /// </summary>
        public static BakedAeroDatabase AnalyzeAndBake(AircraftConfigSpec config)
        {
            var atm = IsaAtmosphereAndTurbulence.Evaluate(config.DesignAltitudeMsl);
            var panels = VlmAndLiftingLineSolver.BuildPanels(config.Wings);

            double sRef = config.ReferenceAreaSqM;
            double cRef = config.ReferenceChordMeters;
            double bRef = config.ReferenceSpanMeters;

            if (config.Wings != null && config.Wings.Count > 0 && !config.Wings[0].IsVerticalFin)
            {
                sRef = config.Wings[0].PlanformArea;
                cRef = config.Wings[0].MeanAerodynamicChord;
                bRef = config.Wings[0].Span;
                config.ReferenceAreaSqM = sRef;
                config.ReferenceChordMeters = cRef;
                config.ReferenceSpanMeters = bRef;
            }

            var db = new BakedAeroDatabase
            {
                AircraftName = config.AircraftName,
                AnalysisTimestampUtc = DateTime.UtcNow.ToString("O"),
                TotalMassKg = config.MassKg,
                CenterOfGravityFrd = config.CenterOfGravityFrd,
                ReferenceAreaSqM = sRef,
                MeanAerodynamicChord = cRef,
                ReferenceSpan = bRef
            };

            // 1. Analyze Rotors via Module 4 (CCBlade BEMT) -> Fit lumped Kt, Kq & J-curves
            AnalyzeRotorsBemt(config, atm, db);

            // 2. Sample 2D Airfoil Polar via Module 1 (-180 deg .. +180 deg)
            AirfoilSpec refAirfoil = (config.Wings != null && config.Wings.Count > 0)
                ? config.Wings[0].Airfoil
                : (config.Rotors.Count > 0 ? config.Rotors[0].BladeAirfoil : AirfoilSpec.FromNaca4Digit("2412"));
            SampleAirfoilPolar(refAirfoil, atm, cRef, config.DesignCruiseSpeedMps, db);

            // 3. Evaluate 3D VLM + Fuselage Surrogate Polar & Stability Derivatives via Modules 2, 3, 5
            AnalyzeStabilityAndDerivatives(config, panels, atm, sRef, cRef, bRef, db);

            // 4. Compute Wrench Control Allocation Pseudoinverse Matrix
            WrenchControlAllocator.BakeAllocatorIntoDatabase(config, db);

            return db;
        }

        private static void AnalyzeRotorsBemt(AircraftConfigSpec config, AtmosphereState atm, BakedAeroDatabase db)
        {
            int nRotors = config.Rotors.Count;
            db.RotorLumpedKt = new double[nRotors];
            db.RotorLumpedKq = new double[nRotors];
            db.RotorHoverRpm = new double[nRotors];

            int numVertical = 0;
            for (int i = 0; i < nRotors; i++)
            {
                if (config.Rotors[i].ThrustAxisFrd.Z < -0.5)
                    numVertical++;
            }

            double targetHoverThrustPerRotor = numVertical > 0
                ? (config.MassKg * IsaAtmosphereAndTurbulence.GravityMps2) / numVertical
                : (config.MassKg * IsaAtmosphereAndTurbulence.GravityMps2 * 0.25);

            for (int i = 0; i < nRotors; i++)
            {
                var r = config.Rotors[i];
                double testRpm = 0.65 * r.MaxRpm;
                var sol = CcBladeBemtSolver.EvaluateRotor(r, Vec3d.Zero, Vec3d.Zero, testRpm, atm, 50.0, false, false);
                double omegaSq = System.Math.Max(1.0, sol.OmegaRadps * sol.OmegaRadps);

                double kt = System.Math.Max(1e-7, sol.ThrustNewtons / omegaSq);
                double kq = System.Math.Max(1e-9, sol.AerodynamicTorqueNm / omegaSq);
                db.RotorLumpedKt[i] = kt;
                db.RotorLumpedKq[i] = kq;

                double hoverOmega = System.Math.Sqrt(targetHoverThrustPerRotor / kt);
                db.RotorHoverRpm[i] = hoverOmega * MathConstants.RadPerSecToRpm;
            }

            // Generate Advance Ratio J curve for primary rotor [0 .. 0.85]
            const int nJ = 18;
            db.AdvanceRatioGrid = new double[nJ];
            db.BemtCtCurve = new double[nJ];
            db.BemtCqCurve = new double[nJ];
            db.BemtEtaCurve = new double[nJ];

            if (nRotors > 0)
            {
                var r0 = config.Rotors[0];
                double rpmRef = 0.75 * r0.MaxRpm;
                double nRev = rpmRef / 60.0;
                double diam = 2.0 * r0.Radius;
                Vec3d axis = r0.ThrustAxisFrd.Normalized();

                for (int k = 0; k < nJ; k++)
                {
                    double jVal = k * 0.05;
                    double vAx = jVal * nRev * diam;
                    var sol = CcBladeBemtSolver.EvaluateRotor(r0, axis * vAx, Vec3d.Zero, rpmRef, atm, 100.0, false, false);
                    db.AdvanceRatioGrid[k] = jVal;
                    db.BemtCtCurve[k] = sol.Ct;
                    db.BemtCqCurve[k] = sol.Cq;
                    db.BemtEtaCurve[k] = sol.EfficiencyEta;
                }
            }
        }

        private static void SampleAirfoilPolar(AirfoilSpec airfoil, AtmosphereState atm, double chord, double speed, BakedAeroDatabase db)
        {
            const int nPts = 73; // -180 to +180 every 5 deg
            db.AirfoilAlphaDegGrid = new double[nPts];
            db.AirfoilClGrid = new double[nPts];
            db.AirfoilCdGrid = new double[nPts];
            db.AirfoilCmGrid = new double[nPts];

            double re = IsaAtmosphereAndTurbulence.ComputeReynolds(speed, chord, atm);
            double mach = IsaAtmosphereAndTurbulence.ComputeMach(speed, atm);

            for (int i = 0; i < nPts; i++)
            {
                double deg = -180.0 + i * 5.0;
                var pt = AirfoilPolarSolver.Evaluate(airfoil, deg * FrameConverter.Deg2Rad, re, mach);
                db.AirfoilAlphaDegGrid[i] = deg;
                db.AirfoilClGrid[i] = pt.Cl;
                db.AirfoilCdGrid[i] = pt.Cd;
                db.AirfoilCmGrid[i] = pt.Cm;
            }
        }

        private static void AnalyzeStabilityAndDerivatives(
            AircraftConfigSpec config,
            List<WingPanelElement> panels,
            AtmosphereState atm,
            double sRef,
            double cRef,
            double bRef,
            BakedAeroDatabase db)
        {
            double vRef = System.Math.Max(5.0, config.DesignCruiseSpeedMps);
            double qDyn = 0.5 * atm.DensityKgPerM3 * vRef * vRef;
            double qS = qDyn * sRef;

            // Sample 3D Aircraft Polar (-30..+30 deg)
            const int nA = 25;
            db.AircraftAlphaDegGrid = new double[nA];
            db.AircraftCLGrid = new double[nA];
            db.AircraftCDGrid = new double[nA];
            db.AircraftCmGrid = new double[nA];

            for (int i = 0; i < nA; i++)
            {
                double deg = -30.0 + i * 2.5;
                double rad = deg * FrameConverter.Deg2Rad;
                EvaluateCombinedAeroState(config, panels, atm, vRef, rad, 0.0, Vec3d.Zero, null,
                    out double cl, out double cd, out _, out _, out double cm, out _);
                db.AircraftAlphaDegGrid[i] = deg;
                db.AircraftCLGrid[i] = cl;
                db.AircraftCDGrid[i] = cd;
                db.AircraftCmGrid[i] = cm;
            }

            // Baseline at alpha = 0, beta = 0
            EvaluateCombinedAeroState(config, panels, atm, vRef, 0.0, 0.0, Vec3d.Zero, null,
                out double cl0, out double cd0, out _, out _, out double cm0, out _);
            db.CL0 = cl0;
            db.CD0 = cd0;
            db.Cm0 = cm0;

            // Central finite difference wrt alpha (+/- 1 deg)
            double dAlpha = 1.0 * FrameConverter.Deg2Rad;
            EvaluateCombinedAeroState(config, panels, atm, vRef, +dAlpha, 0.0, Vec3d.Zero, null,
                out double clAp, out double cdAp, out _, out _, out double cmAp, out _);
            EvaluateCombinedAeroState(config, panels, atm, vRef, -dAlpha, 0.0, Vec3d.Zero, null,
                out double clAm, out _, out _, out _, out double cmAm, out _);

            db.CLAlpha = (clAp - clAm) / (2.0 * dAlpha);
            db.CmAlpha = (cmAp - cmAm) / (2.0 * dAlpha);

            double arEff = (bRef * bRef) / System.Math.Max(0.01, sRef);
            db.OswaldEfficiency = System.Math.Clamp(1.78 * (1.0 - 0.045 * System.Math.Pow(arEff, 0.68)) - 0.64, 0.65, 0.90);
            db.InducedDragFactorK = 1.0 / (System.Math.PI * System.Math.Max(1.5, arEff) * db.OswaldEfficiency);

            // Central finite difference wrt sideslip beta (+/- 1 deg)
            double dBeta = 1.0 * FrameConverter.Deg2Rad;
            EvaluateCombinedAeroState(config, panels, atm, vRef, 0.0, +dBeta, Vec3d.Zero, null,
                out _, out _, out double cyBp, out double clRollBp, out _, out double cnYawBp);
            EvaluateCombinedAeroState(config, panels, atm, vRef, 0.0, -dBeta, Vec3d.Zero, null,
                out _, out _, out double cyBm, out double clRollBm, out _, out double cnYawBm);

            db.CYBeta = (cyBp - cyBm) / (2.0 * dBeta);
            db.ClBeta = (clRollBp - clRollBm) / (2.0 * dBeta);
            db.CnBeta = (cnYawBp - cnYawBm) / (2.0 * dBeta);

            // Dynamic damping derivatives wrt non-dimensional rates p_hat = p*b/(2V), q_hat = q*c/(2V), r_hat = r*b/(2V)
            double omegaStep = 0.25; // rad/s
            double pHat = (omegaStep * bRef) / (2.0 * vRef);
            double qHat = (omegaStep * cRef) / (2.0 * vRef);

            EvaluateCombinedAeroState(config, panels, atm, vRef, 0.0, 0.0, new Vec3d(omegaStep, 0.0, 0.0), null,
                out _, out _, out double cyP, out double clP, out _, out double cnP);
            db.CYp = cyP / pHat;
            db.Clp = clP / pHat;
            db.Cnp = cnP / pHat;

            EvaluateCombinedAeroState(config, panels, atm, vRef, 0.0, 0.0, new Vec3d(0.0, omegaStep, 0.0), null,
                out double clQ, out _, out _, out _, out double cmQ, out _);
            db.CLq = (clQ - cl0) / qHat;
            db.Cmq = (cmQ - cm0) / qHat;

            EvaluateCombinedAeroState(config, panels, atm, vRef, 0.0, 0.0, new Vec3d(0.0, 0.0, omegaStep), null,
                out _, out _, out double cyR, out double clR, out _, out double cnR);
            db.CYr = cyR / pHat;
            db.Clr = clR / pHat;
            db.Cnr = cnR / pHat;

            // Control surface derivatives (Aileron, Elevator, Rudder)
            EvaluateControlDerivatives(config, panels, atm, vRef, cl0, cm0, db);

            // Neutral Point & Static Margin SM = -Cm_alpha / CL_alpha
            if (System.Math.Abs(db.CLAlpha) > 0.1)
            {
                db.StaticMarginFraction = -db.CmAlpha / db.CLAlpha;
                db.NeutralPointXFrd = config.CenterOfGravityFrd.X - db.StaticMarginFraction * cRef;
            }
            else
            {
                db.StaticMarginFraction = 0.0;
                db.NeutralPointXFrd = config.CenterOfGravityFrd.X;
            }

            db.IsLongitudinallyStable = db.CmAlpha < -0.01;
            db.IsDirectionallyStable = db.CnBeta > 0.005;

            // Compute Trim Condition (L = W, Cm = 0)
            db.TrimAirspeedMps = vRef;
            if (System.Math.Abs(db.CLAlpha) > 0.2)
            {
                double clReq = (config.MassKg * IsaAtmosphereAndTurbulence.GravityMps2) / System.Math.Max(1.0, qS);
                double det = db.CLAlpha * db.CmDeltaE - db.CLDeltaE * db.CmAlpha;
                if (System.Math.Abs(det) > 1e-4)
                {
                    double rhsL = clReq - db.CL0;
                    double rhsM = -db.Cm0;
                    double alphaTrimRad = (rhsL * db.CmDeltaE - db.CLDeltaE * rhsM) / det;
                    double deTrimRad = (db.CLAlpha * rhsM - rhsL * db.CmAlpha) / det;
                    db.TrimAlphaDeg = alphaTrimRad * FrameConverter.Rad2Deg;
                    db.TrimElevatorDeg = deTrimRad * FrameConverter.Rad2Deg;
                }
                else
                {
                    db.TrimAlphaDeg = ((clReq - db.CL0) / db.CLAlpha) * FrameConverter.Rad2Deg;
                }
                double cdTrim = db.CD0 + db.InducedDragFactorK * clReq * clReq;
                double dragTrim = qS * cdTrim;
                double maxForwardThrust = System.Math.Max(10.0, config.MassKg * 8.0);
                db.TrimThrottleNormalized = System.Math.Clamp(dragTrim / maxForwardThrust, 0.1, 0.95);
            }
            else
            {
                // Multirotor hover trim
                db.TrimAirspeedMps = 0.0;
                db.TrimAlphaDeg = 0.0;
                db.TrimElevatorDeg = 0.0;
                db.TrimThrottleNormalized = db.RotorHoverRpm.Length > 0 && config.Rotors.Count > 0
                    ? System.Math.Clamp(db.RotorHoverRpm[0] / config.Rotors[0].MaxRpm, 0.1, 0.95)
                    : 0.52;
            }
        }

        private static void EvaluateControlDerivatives(
            AircraftConfigSpec config,
            List<WingPanelElement> panels,
            AtmosphereState atm,
            double vRef,
            double cl0,
            double cm0,
            BakedAeroDatabase db)
        {
            if (config.Wings == null || config.Wings.Count == 0) return;

            double[] ctrl = new double[16];
            for (int wIdx = 0; wIdx < config.Wings.Count; wIdx++)
            {
                var w = config.Wings[wIdx];
                if (w.ControlType == ControlSurfaceType.None || w.ActuatorChannelIndex < 0 || w.ActuatorChannelIndex >= 16)
                    continue;

                Array.Clear(ctrl, 0, ctrl.Length);
                ctrl[w.ActuatorChannelIndex] = 0.5; // 50% deflection
                double dRad = 0.5 * w.MaxDeflectionDeg * FrameConverter.Deg2Rad;
                if (dRad < 1e-4) continue;

                EvaluateCombinedAeroState(config, panels, atm, vRef, 0.0, 0.0, Vec3d.Zero, ctrl,
                    out double cl, out _, out double cy, out double clRoll, out double cm, out double cnYaw);

                switch (w.ControlType)
                {
                    case ControlSurfaceType.Aileron:
                        db.ClDeltaA = clRoll / dRad;
                        db.CnDeltaA = cnYaw / dRad;
                        break;
                    case ControlSurfaceType.Elevator:
                        db.CLDeltaE = (cl - cl0) / dRad;
                        db.CmDeltaE = (cm - cm0) / dRad;
                        break;
                    case ControlSurfaceType.Rudder:
                        db.CYDeltaR = cy / dRad;
                        db.ClDeltaR = clRoll / dRad;
                        db.CnDeltaR = cnYaw / dRad;
                        break;
                }
            }
        }

        private static void EvaluateCombinedAeroState(
            AircraftConfigSpec config,
            List<WingPanelElement> panels,
            AtmosphereState atm,
            double vMag,
            double alphaRad,
            double betaRad,
            Vec3d omegaBodyFrd,
            double[]? controls,
            out double cl,
            out double cd,
            out double cy,
            out double clRoll,
            out double cmPitch,
            out double cnYaw)
        {
            double cosA = System.Math.Cos(alphaRad);
            double sinA = System.Math.Sin(alphaRad);
            double cosB = System.Math.Cos(betaRad);
            double sinB = System.Math.Sin(betaRad);

            Vec3d vBodyFrd = new Vec3d(
                vMag * cosA * cosB,
                vMag * sinB,
                vMag * sinA * cosB);

            var wingRes = VlmAndLiftingLineSolver.Evaluate(
                config.Wings, panels, vBodyFrd, omegaBodyFrd, config.CenterOfGravityFrd,
                atm, controls, null, null, enable3DVlmWakeInduction: true);

            var fusRes = FuselageAndParasiticSolver.Evaluate(
                config.Fuselage, vBodyFrd, config.CenterOfGravityFrd,
                config.ReferenceAreaSqM, config.ReferenceChordMeters, config.ReferenceSpanMeters, atm);

            Vec3d fTotalFrd = wingRes.TotalForceFrd + fusRes.ForceFrd;
            Vec3d mTotalFrd = wingRes.TotalMomentAboutCgFrd + fusRes.MomentAboutCgFrd;

            double qDyn = 0.5 * atm.DensityKgPerM3 * vMag * vMag;
            double qS = System.Math.Max(1e-4, qDyn * config.ReferenceAreaSqM);
            double cRef = System.Math.Max(0.05, config.ReferenceChordMeters);
            double bRef = System.Math.Max(0.10, config.ReferenceSpanMeters);

            // Transform FRD forces to Wind/Stability axes Lift & Drag
            double lift = -fTotalFrd.Z * cosA + fTotalFrd.X * sinA;
            double drag = -fTotalFrd.X * cosA - fTotalFrd.Z * sinA;

            cl = lift / qS;
            cd = System.Math.Max(fusRes.ParasiticCd0, drag / qS);
            cy = fTotalFrd.Y / qS;
            clRoll = mTotalFrd.X / (qS * bRef);
            cmPitch = mTotalFrd.Y / (qS * cRef);
            cnYaw = mTotalFrd.Z / (qS * bRef);
        }
    }
}
