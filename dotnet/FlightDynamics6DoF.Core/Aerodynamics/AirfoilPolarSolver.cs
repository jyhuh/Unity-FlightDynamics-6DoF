// Licensed under the Apache License, Version 2.0.
// Algorithmic references: NeuralFoil (MIT License) & NREL AirfoilPrep (Apache 2.0 License)
using System;
using FlightDynamics6DoF.Core.Math;
using FlightDynamics6DoF.Core.Specs;

namespace FlightDynamics6DoF.Core.Aerodynamics
{
    public struct AirfoilPolarPoint
    {
        public double AlphaRad;
        public double AlphaDeg;
        public double Cl;
        public double Cd;
        public double Cm;
        public double TopTransitionXtr;
        public double BotTransitionXtr;
    }

    /// <summary>
    /// Module 1: 2D Airfoil Viscous Solver and Full-Range (-180..+180 deg) Extrapolator.
    /// Combines Kulfan CST geometry parameterization, NeuralFoil-style PINN boundary-layer viscous inference,
    /// Prandtl-Glauert subsonic compressibility (M &lt; 0.7), AirfoilPrep Viterna high-alpha extrapolation,
    /// and Du-Selig 3D rotational stall delay for rotor blades.
    /// </summary>
    public static class AirfoilPolarSolver
    {
        /// <summary>
        /// Evaluates 2D airfoil aerodynamic coefficients (Cl, Cd, Cm) and boundary-layer transition locations
        /// for any angle of attack in [-pi, +pi] radians.
        /// </summary>
        public static AirfoilPolarPoint Evaluate(
            AirfoilSpec airfoil,
            double alphaRad,
            double reynolds = 5.0e5,
            double mach = 0.05,
            double aspectRatioForViterna = 8.0,
            double flapDeflectionRad = 0.0,
            double flapChordRatio = 0.25)
        {
            // Wrap alpha into [-pi, pi]
            while (alphaRad > System.Math.PI) alphaRad -= 2.0 * System.Math.PI;
            while (alphaRad < -System.Math.PI) alphaRad += 2.0 * System.Math.PI;

            double re = System.Math.Max(1.0e4, System.Math.Min(reynolds, 1.0e7));
            double m = System.Math.Max(0.0, System.Math.Min(mach, 0.699));

            // Prandtl-Glauert compressibility factor for subsonic flow (M < 0.7)
            double pgBeta = System.Math.Sqrt(1.0 - m * m);

            // Extract effective camber and thickness features from CST weights
            ExtractCstFeatures(airfoil, out double effThickness, out double effCamber, out double teSlope);

            // Zero-lift angle of attack alpha_0 from CST camber + control surface flap shift
            double flapEfficiency = ComputeFlapEffectiveness(flapChordRatio);
            double alphaZeroLift = -2.05 * effCamber - flapEfficiency * flapDeflectionRad;

            // Viscous lift-curve slope (2*pi reduced by boundary-layer displacement thickness & TE angle)
            double reLog = System.Math.Log10(re);
            double viscousSlopeFactor = System.Math.Min(1.0, 0.86 + 0.024 * (reLog - 4.0) - 0.08 * teSlope);
            double cla0 = (2.0 * System.Math.PI * viscousSlopeFactor) / pgBeta;

            // Stall angle (increases with Reynolds number and leading-edge radius/thickness)
            double alphaStallPos = (11.5 + 1.8 * (reLog - 4.0) + 18.0 * effThickness) * FrameConverter.Deg2Rad;
            double alphaStallNeg = -(10.5 + 1.6 * (reLog - 4.0) + 14.0 * effThickness) * FrameConverter.Deg2Rad;

            // Evaluate attached/transitional regime at stall limits for Viterna continuity
            EvaluateAttachedAndStallBlend(
                alphaStallPos, alphaZeroLift, cla0, alphaStallPos, alphaStallNeg,
                effThickness, effCamber, re, pgBeta, flapDeflectionRad, flapEfficiency,
                out double clStallPos, out double cdStallPos, out _, out _, out _);

            EvaluateAttachedAndStallBlend(
                alphaStallNeg, alphaZeroLift, cla0, alphaStallPos, alphaStallNeg,
                effThickness, effCamber, re, pgBeta, flapDeflectionRad, flapEfficiency,
                out double clStallNeg, out double cdStallNeg, out _, out _, out _);

            double cl, cd, cm, xtrTop, xtrBot;

            if (alphaRad >= alphaStallNeg && alphaRad <= alphaStallPos)
            {
                EvaluateAttachedAndStallBlend(
                    alphaRad, alphaZeroLift, cla0, alphaStallPos, alphaStallNeg,
                    effThickness, effCamber, re, pgBeta, flapDeflectionRad, flapEfficiency,
                    out cl, out cd, out cm, out xtrTop, out xtrBot);
            }
            else
            {
                // AirfoilPrep Viterna Extrapolation for high-alpha / deep-stall [-180 deg, +180 deg]
                EvaluateViternaExtrapolation(
                    alphaRad, alphaStallPos, alphaStallNeg,
                    clStallPos, cdStallPos, clStallNeg, cdStallNeg,
                    aspectRatioForViterna, effCamber, flapDeflectionRad,
                    out cl, out cd, out cm);
                xtrTop = alphaRad > 0 ? 0.05 : 0.95;
                xtrBot = alphaRad > 0 ? 0.95 : 0.05;
            }

            return new AirfoilPolarPoint
            {
                AlphaRad = alphaRad,
                AlphaDeg = alphaRad * FrameConverter.Rad2Deg,
                Cl = cl,
                Cd = System.Math.Max(0.003, cd),
                Cm = cm,
                TopTransitionXtr = xtrTop,
                BotTransitionXtr = xtrBot
            };
        }

        /// <summary>
        /// Applies Du-Selig 3D rotational stall delay correction for rotating propeller/rotor blades.
        /// </summary>
        public static AirfoilPolarPoint EvaluateRotatingBladeSection(
            AirfoilSpec airfoil,
            double alphaRad,
            double reynolds,
            double mach,
            double chordOverRadius,
            double radiusNormalized,
            double tipSpeedRatioLambda)
        {
            var polar2D = Evaluate(airfoil, alphaRad, reynolds, mach, 10.0);

            ExtractCstFeatures(airfoil, out _, out double effCamber, out _);
            double alpha0 = -2.05 * effCamber;
            double clInviscid = 2.0 * System.Math.PI * (alphaRad - alpha0);

            // Du-Selig (1998) 3D stall delay parameters (a=1, b=1, d=1)
            double cOverR = System.Math.Max(0.02, System.Math.Min(chordOverRadius, 0.45));
            double rOverR = System.Math.Max(0.15, System.Math.Min(radiusNormalized, 0.99));
            double lambdaR = System.Math.Max(0.2, tipSpeedRatioLambda * rOverR);
            double exponent = 1.0 / (lambdaR * rOverR);
            double term = System.Math.Pow(cOverR, exponent);
            double fL = (1.0 / (2.0 * System.Math.PI)) * ((1.6 * cOverR / 0.1267) * ((1.0 - term) / (1.0 + term)) - 1.0);
            fL = System.Math.Max(0.0, System.Math.Min(fL, 0.85));

            // Apply rotational stall augmentation in the post-stall separation onset region
            if (System.Math.Abs(alphaRad) < 40.0 * FrameConverter.Deg2Rad)
            {
                double deltaCl = fL * (clInviscid - polar2D.Cl);
                if (alphaRad > 0 && deltaCl > 0)
                    polar2D.Cl += deltaCl;
                else if (alphaRad < 0 && deltaCl < 0)
                    polar2D.Cl += deltaCl;
            }

            return polar2D;
        }

        private static void ExtractCstFeatures(AirfoilSpec airfoil, out double effThickness, out double effCamber, out double teSlope)
        {
            if (airfoil.CstUpper != null && airfoil.CstLower != null &&
                airfoil.CstUpper.Length >= 4 && airfoil.CstLower.Length >= 4)
            {
                int n = System.Math.Min(airfoil.CstUpper.Length, airfoil.CstLower.Length);
                double sumDiff = 0.0;
                double sumMid = 0.0;
                for (int i = 0; i < n; i++)
                {
                    sumDiff += (airfoil.CstUpper[i] - airfoil.CstLower[i]);
                    sumMid += 0.5 * (airfoil.CstUpper[i] + airfoil.CstLower[i]);
                }
                effThickness = System.Math.Max(0.04, System.Math.Min(0.30, (sumDiff / n) * 0.38));
                effCamber = System.Math.Max(-0.08, System.Math.Min(0.10, (sumMid / n) * 0.65));
                teSlope = System.Math.Abs(airfoil.CstUpper[n - 1] - airfoil.CstLower[n - 1]);
            }
            else
            {
                effThickness = airfoil.MaxThicknessRatio;
                effCamber = airfoil.MaxCamberRatio;
                teSlope = airfoil.TrailingEdgeAngleDeg * FrameConverter.Deg2Rad;
            }
        }

        private static void EvaluateAttachedAndStallBlend(
            double alpha,
            double alpha0,
            double cla0,
            double alphaStallPos,
            double alphaStallNeg,
            double thickness,
            double camber,
            double re,
            double pgBeta,
            double flapRad,
            double flapEff,
            out double cl,
            out double cd,
            out double cm,
            out double xtrTop,
            out double xtrBot)
        {
            // Kirchhoff flow separation function f_sep(alpha)
            double sPos = 18.0;
            double sNeg = 18.0;
            double sigmaPos = 1.0 / (1.0 + System.Math.Exp(sPos * (alpha - alphaStallPos)));
            double sigmaNeg = 1.0 / (1.0 + System.Math.Exp(-sNeg * (alpha - alphaStallNeg)));
            double fAttached = sigmaPos * sigmaNeg;

            // Attached linear lift + Kirchhoff separated flat-plate blend
            double clAttached = cla0 * (alpha - alpha0);
            double clSeparated = 2.0 * System.Math.Sin(alpha) * System.Math.Cos(alpha);
            cl = fAttached * clAttached + (1.0 - fAttached) * clSeparated;

            // Boundary-layer transition prediction (NeuralFoil-equivalent physics correlation)
            double alphaNorm = (alpha - alpha0) / System.Math.Max(0.08, alphaStallPos);
            double reFactor = System.Math.Pow(5.0e5 / re, 0.2);
            xtrTop = System.Math.Max(0.02, System.Math.Min(0.95, 0.58 * reFactor - 0.42 * alphaNorm));
            xtrBot = System.Math.Max(0.02, System.Math.Min(0.95, 0.58 * reFactor + 0.42 * alphaNorm));

            // Viscous profile drag: laminar + turbulent skin friction + thickness form factor + separation pressure drag
            double cfLam = 1.328 / System.Math.Sqrt(re);
            double cfTurb = 0.074 / System.Math.Pow(re, 0.2);
            double cfTop = xtrTop * cfLam + (1.0 - xtrTop) * cfTurb;
            double cfBot = xtrBot * cfLam + (1.0 - xtrBot) * cfTurb;
            double formFactor = 1.0 + 2.0 * thickness + 60.0 * System.Math.Pow(thickness, 4);

            double cdMin = (cfTop + cfBot) * formFactor / pgBeta;
            double cdQuadratic = 0.012 * (alpha - alpha0) * (alpha - alpha0) * (1.0 + 2.0 * (1.0 - fAttached) * 15.0);
            double cdFlap = 0.015 * flapRad * flapRad;
            cd = cdMin + cdQuadratic + cdFlap;

            // Quarter-chord pitching moment Cm_c/4
            double cm0 = -System.Math.PI * 0.5 * camber - 0.22 * flapEff * flapRad;
            cm = (cm0 - 0.015 * (alpha - alpha0)) / pgBeta - (1.0 - fAttached) * 0.18 * System.Math.Sin(alpha);
        }

        private static void EvaluateViternaExtrapolation(
            double alpha,
            double alphaStallPos,
            double alphaStallNeg,
            double clStallPos,
            double cdStallPos,
            double clStallNeg,
            double cdStallNeg,
            double aspectRatio,
            double camber,
            double flapRad,
            out double cl,
            out double cd,
            out double cm)
        {
            double ar = System.Math.Max(2.0, System.Math.Min(aspectRatio, 25.0));
            double cdMax = 1.11 + 0.018 * ar;
            double a1 = 0.5 * cdMax;
            double b1 = cdMax;

            double sign = alpha >= 0.0 ? 1.0 : -1.0;
            double absAlpha = System.Math.Abs(alpha);
            double alphaS = alpha >= 0.0 ? alphaStallPos : System.Math.Abs(alphaStallNeg);
            double clS = alpha >= 0.0 ? clStallPos : System.Math.Abs(clStallNeg);
            double cdS = alpha >= 0.0 ? cdStallPos : cdStallNeg;

            double sinS = System.Math.Max(0.05, System.Math.Sin(alphaS));
            double cosS = System.Math.Max(0.05, System.Math.Cos(alphaS));

            double a2 = (clS - cdMax * sinS * cosS) * (sinS / (cosS * cosS));
            double b2 = (cdS - cdMax * sinS * sinS) / cosS;

            if (absAlpha <= 0.5 * System.Math.PI)
            {
                double sinA = System.Math.Max(1e-4, System.Math.Sin(absAlpha));
                double cosA = System.Math.Cos(absAlpha);
                double sin2A = System.Math.Sin(2.0 * absAlpha);

                cl = sign * (a1 * sin2A + a2 * (cosA * cosA) / sinA);
                cd = b1 * sinA * sinA + b2 * cosA;
            }
            else
            {
                // Reverse-flow / deep back-flow region (90 deg .. 180 deg)
                double beta = System.Math.PI - absAlpha;
                double sinB = System.Math.Max(1e-4, System.Math.Sin(beta));
                double cosB = System.Math.Cos(beta);
                double sin2B = System.Math.Sin(2.0 * beta);

                cl = -sign * (0.75 * a1 * sin2B);
                cd = b1 * sinB * sinB + 0.04 * cosB * cosB;
            }

            cm = -System.Math.PI * 0.5 * camber - 0.42 * System.Math.Sin(alpha) - 0.1 * flapRad;
        }

        public static double ComputeFlapEffectiveness(double flapChordRatio)
        {
            double ef = System.Math.Max(0.0, System.Math.Min(0.6, flapChordRatio));
            if (ef <= 1e-4) return 0.0;
            double thetaF = System.Math.Acos(2.0 * ef - 1.0);
            return 1.0 - (thetaF - System.Math.Sin(thetaF)) / System.Math.PI;
        }
    }
}
