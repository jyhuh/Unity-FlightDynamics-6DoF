// Licensed under the Apache License, Version 2.0.
// Algorithmic references: AeroSandbox Slender Body Theory & Component Build-up (MIT License)
using System;
using FlightDynamics6DoF.Core.Math;
using FlightDynamics6DoF.Core.Specs;

namespace FlightDynamics6DoF.Core.Aerodynamics
{
    public struct FuselageAeroResult
    {
        public double ParasiticCd0;
        public double CmAlphaFuselagePerRad;
        public double CnBetaFuselagePerRad;
        public Vec3d ForceFrd;
        public Vec3d MomentAboutCgFrd;
    }

    /// <summary>
    /// Module 3: Fuselage and Non-Lifting Body Aerodynamic Solver.
    /// Combines Multhopp Slender Body Theory (dS/dx destabilizing pitching/yawing moment),
    /// Component Build-up parasitic drag (Cf * FF * Q * Swet / Sref),
    /// and High-Alpha Cross-Flow Principle bluff-body drag for multirotors and deep-stall flight.
    /// </summary>
    public static class FuselageAndParasiticSolver
    {
        public static FuselageAeroResult Evaluate(
            FuselageSpec fuselage,
            Vec3d velocityRelativeAirFrd,
            Vec3d cgFrd,
            double referenceAreaSqM,
            double referenceChordM,
            double referenceSpanM,
            AtmosphereState atm)
        {
            double sRef = System.Math.Max(0.01, referenceAreaSqM);
            double cRef = System.Math.Max(0.05, referenceChordM);
            double bRef = System.Math.Max(0.10, referenceSpanM);

            double vMag = velocityRelativeAirFrd.Magnitude;
            double reL = IsaAtmosphereAndTurbulence.ComputeReynolds(System.Math.Max(2.0, vMag), fuselage.Length, atm);
            double mach = IsaAtmosphereAndTurbulence.ComputeMach(vMag, atm);

            // 1. Component Build-up Parasitic Drag CD0 = (Cf * FF * Q * Swet) / Sref + MiscDeltaCd0
            double cfTurb = 0.455 / (System.Math.Pow(System.Math.Log10(reL), 2.58) * System.Math.Pow(1.0 + 0.144 * mach * mach, 0.65));
            double finenessRatio = fuselage.Length / System.Math.Max(0.02, fuselage.MaxDiameter);
            double fClamped = System.Math.Clamp(finenessRatio, 1.5, 20.0);
            double formFactor = 1.0 + 60.0 / System.Math.Pow(fClamped, 3) + fClamped / 400.0;

            double cd0Fuselage = (cfTurb * formFactor * fuselage.InterferenceFactorQ * fuselage.WettedArea) / sRef + fuselage.MiscDeltaCd0;

            // 2. Slender Body Theory (Multhopp / Munk integral of dS/dx) -> destabilizing Cm_alpha_fus > 0 and Cn_beta_fus < 0
            double volumeApprox = EstimateVolumeFromStations(fuselage);
            double apparentMassK2MinusK1 = System.Math.Min(0.95, 1.0 - 0.55 / System.Math.Pow(fClamped, 0.7));
            double cmAlphaFus = (2.0 * apparentMassK2MinusK1 * volumeApprox) / (sRef * cRef);
            double cnBetaFus = -(2.0 * apparentMassK2MinusK1 * volumeApprox) / (sRef * bRef);

            if (double.IsNaN(vMag) || vMag < 0.01)
            {
                return new FuselageAeroResult
                {
                    ParasiticCd0 = double.IsNaN(cd0Fuselage) ? 0.02 : cd0Fuselage,
                    CmAlphaFuselagePerRad = double.IsNaN(cmAlphaFus) ? 0.0 : cmAlphaFus,
                    CnBetaFuselagePerRad = double.IsNaN(cnBetaFus) ? 0.0 : cnBetaFus,
                    ForceFrd = Vec3d.Zero,
                    MomentAboutCgFrd = Vec3d.Zero
                };
            }

            double qDyn = 0.5 * atm.DensityKgPerM3 * vMag * vMag;
            double alpha = System.Math.Atan2(velocityRelativeAirFrd.Z, System.Math.Max(0.1, System.Math.Abs(velocityRelativeAirFrd.X)));
            double beta = System.Math.Asin(System.Math.Clamp(velocityRelativeAirFrd.Y / vMag, -1.0, 1.0));

            // 3. Cross-Flow Principle + Skin Friction Forces along FRD axes
            double u = velocityRelativeAirFrd.X;
            double v = velocityRelativeAirFrd.Y;
            double w = velocityRelativeAirFrd.Z;

            double uSign = double.IsNaN(u) || System.Math.Abs(u) < 1e-6 ? 1.0 : System.Math.Sign(u);
            double fxStreamlined = -qDyn * sRef * cd0Fuselage * uSign * (u * u) / (vMag * vMag);
            double fxBluff = -0.5 * atm.DensityKgPerM3 * fuselage.ProjectedAreasFrd.X * fuselage.BluffDragCoefficientsFrd.X * u * System.Math.Abs(u);
            double fx = 0.5 * fxStreamlined + 0.5 * fxBluff;

            double fy = -0.5 * atm.DensityKgPerM3 * fuselage.ProjectedAreasFrd.Y * fuselage.BluffDragCoefficientsFrd.Y * v * System.Math.Abs(v);
            double fz = -0.5 * atm.DensityKgPerM3 * fuselage.ProjectedAreasFrd.Z * fuselage.BluffDragCoefficientsFrd.Z * w * System.Math.Abs(w);

            Vec3d forceFrd = new Vec3d(fx, fy, fz);

            // Slender-body destabilizing pitching and yawing moments
            double pitchMoment = qDyn * sRef * cRef * (cmAlphaFus * System.Math.Sin(2.0 * alpha) * 0.5);
            double yawMoment = qDyn * sRef * bRef * (cnBetaFus * System.Math.Sin(2.0 * beta) * 0.5);

            Vec3d arm = fuselage.CenterPositionFrd - cgFrd;
            Vec3d momentAboutCg = Vec3d.Cross(arm, forceFrd) + new Vec3d(0.0, pitchMoment, yawMoment);

            return new FuselageAeroResult
            {
                ParasiticCd0 = cd0Fuselage,
                CmAlphaFuselagePerRad = cmAlphaFus,
                CnBetaFuselagePerRad = cnBetaFus,
                ForceFrd = forceFrd,
                MomentAboutCgFrd = momentAboutCg
            };
        }

        private static double EstimateVolumeFromStations(FuselageSpec fuselage)
        {
            if (fuselage.AxialStationsNormalized != null &&
                fuselage.CrossSectionAreas != null &&
                fuselage.AxialStationsNormalized.Length >= 2 &&
                fuselage.AxialStationsNormalized.Length == fuselage.CrossSectionAreas.Length)
            {
                double vol = 0.0;
                for (int i = 0; i < fuselage.AxialStationsNormalized.Length - 1; i++)
                {
                    double dx = (fuselage.AxialStationsNormalized[i + 1] - fuselage.AxialStationsNormalized[i]) * fuselage.Length;
                    double avgArea = 0.5 * (fuselage.CrossSectionAreas[i] + fuselage.CrossSectionAreas[i + 1]);
                    vol += avgArea * System.Math.Max(0.0, dx);
                }
                return System.Math.Max(1e-4, vol);
            }

            double r = 0.5 * fuselage.MaxDiameter;
            return 0.65 * System.Math.PI * r * r * fuselage.Length;
        }
    }
}
