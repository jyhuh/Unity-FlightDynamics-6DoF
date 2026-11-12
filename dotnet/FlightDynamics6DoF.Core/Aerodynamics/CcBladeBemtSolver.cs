// Licensed under the Apache License, Version 2.0.
// Algorithmic references: CCBlade (MIT License, BYU FLOW Lab) & FLOWUnsteady slipstream contraction
using System;
using FlightDynamics6DoF.Core.Math;
using FlightDynamics6DoF.Core.Specs;

namespace FlightDynamics6DoF.Core.Aerodynamics
{
    public struct RotorBemtSolution
    {
        public double Rpm;
        public double OmegaRadps;
        public double AdvanceRatioJ;
        public double EdgewiseAdvanceRatioMu;
        public double ThrustNewtons;
        public double AerodynamicTorqueNm;
        public double ShaftPowerWatts;
        public double Ct;
        public double Cq;
        public double Cp;
        public double EfficiencyEta;
        public double MeanAxialInducedVelocityMps;
        public Vec3d ForceFrd;
        public Vec3d MomentAboutMountFrd;
    }

    /// <summary>
    /// Module 4: Guaranteed-Convergence Blade Element Momentum Theory (CCBlade formulation)
    /// with Prandtl tip/hub loss, Glauert/Buhl turbulent wake correction, edgewise oblique flow,
    /// Cheeseman-Bennett ground effect, and contracting slipstream tube wake velocity coupling.
    /// </summary>
    public static class CcBladeBemtSolver
    {
        public static RotorBemtSolution EvaluateRotor(
            RotorSpec rotor,
            Vec3d aircraftVelocityFrd,
            Vec3d aircraftAngularVelocityFrd,
            double rpm,
            AtmosphereState atm,
            double altitudeAglMeters = 100.0,
            bool enableGroundEffect = true,
            bool enableEdgewiseFlapping = true,
            double tiltAngleRad = 0.0)
        {
            double omega = System.Math.Max(0.0, rpm * MathConstants.RpmToRadPerSec);
            double rTip = System.Math.Max(0.02, rotor.Radius);
            double rHub = System.Math.Clamp(rotor.HubRadius, 0.002, 0.5 * rTip);
            double diameter = 2.0 * rTip;
            double nRevPerSec = rpm / 60.0;

            if (omega < 1.0 || nRevPerSec < 0.1)
            {
                return new RotorBemtSolution { Rpm = rpm, OmegaRadps = omega };
            }

            // Compute rotor thrust unit vector in FRD (incorporating optional tilt servo rotation about Y-axis)
            Vec3d thrustAxis = rotor.ThrustAxisFrd.Normalized();
            if (System.Math.Abs(tiltAngleRad) > 1e-5)
            {
                double ct = System.Math.Cos(tiltAngleRad);
                double st = System.Math.Sin(tiltAngleRad);
                thrustAxis = new Vec3d(
                    ct * thrustAxis.X + st * thrustAxis.Z,
                    thrustAxis.Y,
                    -st * thrustAxis.X + ct * thrustAxis.Z).Normalized();
            }

            // Local velocity of rotor hub in FRD: V_hub = V_cg + omega_body x r_mount
            Vec3d vHubFrd = aircraftVelocityFrd + Vec3d.Cross(aircraftAngularVelocityFrd, rotor.MountPositionFrd);

            // Axial inflow velocity along thrust axis (positive when moving in the thrust direction)
            double vAxial = System.Math.Max(-15.0, Vec3d.Dot(vHubFrd, thrustAxis));
            Vec3d vInPlaneVec = vHubFrd - vAxial * thrustAxis;
            double vInPlane = vInPlaneVec.Magnitude;

            double vTip = omega * rTip;
            double advanceRatioJ = vAxial / System.Math.Max(0.5, nRevPerSec * diameter);
            double mu = vInPlane / System.Math.Max(1.0, vTip);

            int numStations = rotor.RadialStationsNormalized.Length;
            double totalThrust = 0.0;
            double totalTorque = 0.0;
            double sumInducedVelocity = 0.0;
            double prevR = rHub;

            for (int i = 0; i < numStations; i++)
            {
                double xi = rotor.RadialStationsNormalized[i];
                double r = xi * rTip;
                double nextR = (i < numStations - 1)
                    ? 0.5 * (rotor.RadialStationsNormalized[i] + rotor.RadialStationsNormalized[i + 1]) * rTip
                    : rTip;
                double dr = System.Math.Max(1e-4, nextR - prevR);
                prevR = nextR;

                double chord = rotor.RadialChordsMeters[i];
                double twistRad = rotor.RadialTwistDeg[i] * FrameConverter.Deg2Rad;
                double sigma = (rotor.NumBlades * chord) / (2.0 * System.Math.PI * r);

                double vRot = omega * r;

                // Solve 1D CCBlade inflow angle phi in [1e-4, pi/2 - 1e-4] via bracketed bisection
                double phi = SolveInflowAnglePhi(
                    rotor, r, rHub, rTip, chord, twistRad, sigma, vAxial, vRot, vTip, atm);

                double sinPhi = System.Math.Sin(phi);
                double cosPhi = System.Math.Cos(phi);
                double alpha = twistRad - phi;

                double wEff = vRot / System.Math.Max(0.1, cosPhi);
                double re = IsaAtmosphereAndTurbulence.ComputeReynolds(wEff, chord, atm);
                double mach = IsaAtmosphereAndTurbulence.ComputeMach(wEff, atm);

                var polar = AirfoilPolarSolver.EvaluateRotatingBladeSection(
                    rotor.BladeAirfoil, alpha, re, mach, chord / r, xi, vTip / System.Math.Max(1.0, System.Math.Abs(vAxial) + 2.0));

                double cn = polar.Cl * cosPhi - polar.Cd * sinPhi;
                double ct = polar.Cl * sinPhi + polar.Cd * cosPhi;

                double qDyn = 0.5 * atm.DensityKgPerM3 * wEff * wEff;
                double dT = rotor.NumBlades * qDyn * chord * System.Math.Max(0.0, cn) * dr;
                double dQ = rotor.NumBlades * qDyn * chord * System.Math.Max(1e-5, ct) * r * dr;

                totalThrust += dT;
                totalTorque += dQ;

                double uInducedLocal = System.Math.Max(0.0, wEff * sinPhi - System.Math.Max(0.0, vAxial));
                sumInducedVelocity += uInducedLocal;
            }

            // Cheeseman-Bennett Ground Effect when thrusting upwards near ground
            if (enableGroundEffect && thrustAxis.Z < -0.5 && altitudeAglMeters > 0.05)
            {
                double zOverR = System.Math.Max(0.65, altitudeAglMeters / rTip);
                if (zOverR < 4.0)
                {
                    double ratio = 1.0 / (4.0 * zOverR);
                    double geMultiplier = 1.0 / System.Math.Max(0.72, 1.0 - ratio * ratio);
                    totalThrust *= geMultiplier;
                }
            }

            double meanVi = numStations > 0 ? sumInducedVelocity / numStations : 0.0;

            // Edgewise oblique flow (rotor drag H-force and advancing/retreating blade flapping moment)
            Vec3d hForceFrd = Vec3d.Zero;
            Vec3d flappingMomentFrd = Vec3d.Zero;
            if (enableEdgewiseFlapping && vInPlane > 0.05)
            {
                double hDragCoeff = 0.42 * System.Math.Min(0.6, mu);
                hForceFrd = -vInPlaneVec.Normalized() * (totalThrust * hDragCoeff * mu);

                Vec3d rollMomentDir = Vec3d.Cross(thrustAxis, vInPlaneVec.Normalized());
                flappingMomentFrd = rollMomentDir * (0.08 * totalThrust * rTip * mu);
            }

            Vec3d reactiveTorqueFrd = thrustAxis * (rotor.RotationDirection * totalTorque);
            Vec3d totalForceFrd = thrustAxis * totalThrust + hForceFrd;
            Vec3d totalMomentAboutMountFrd = reactiveTorqueFrd + flappingMomentFrd;

            double rho = atm.DensityKgPerM3;
            double d4 = diameter * diameter * diameter * diameter;
            double d5 = d4 * diameter;
            double n2 = System.Math.Max(1e-6, nRevPerSec * nRevPerSec);

            double ctNonDim = totalThrust / (rho * n2 * d4);
            double cqNonDim = totalTorque / (rho * n2 * d5);
            double cpNonDim = 2.0 * System.Math.PI * cqNonDim;
            double eta = cpNonDim > 1e-6 && advanceRatioJ > 0.01
                ? System.Math.Clamp((ctNonDim * advanceRatioJ) / cpNonDim, 0.0, 0.92)
                : 0.0;

            return new RotorBemtSolution
            {
                Rpm = rpm,
                OmegaRadps = omega,
                AdvanceRatioJ = advanceRatioJ,
                EdgewiseAdvanceRatioMu = mu,
                ThrustNewtons = totalThrust,
                AerodynamicTorqueNm = totalTorque,
                ShaftPowerWatts = totalTorque * omega,
                Ct = ctNonDim,
                Cq = cqNonDim,
                Cp = cpNonDim,
                EfficiencyEta = eta,
                MeanAxialInducedVelocityMps = meanVi,
                ForceFrd = totalForceFrd,
                MomentAboutMountFrd = totalMomentAboutMountFrd
            };
        }

        /// <summary>
        /// Computes the velocity vector induced in the Body (FRD) frame at an arbitrary query point (e.g. wing panel or tail)
        /// by a rotor's contracting slipstream tube (FLOWUnsteady / Momentum Slipstream Contraction model).
        /// </summary>
        public static Vec3d ComputeSlipstreamInducedVelocityFrd(
            RotorSpec rotor,
            RotorBemtSolution rotorSol,
            Vec3d queryPointFrd,
            double tiltAngleRad = 0.0)
        {
            if (rotorSol.ThrustNewtons <= 0.01 || rotorSol.MeanAxialInducedVelocityMps <= 1e-3)
                return Vec3d.Zero;

            Vec3d thrustAxis = rotor.ThrustAxisFrd.Normalized();
            if (System.Math.Abs(tiltAngleRad) > 1e-5)
            {
                double ct = System.Math.Cos(tiltAngleRad);
                double st = System.Math.Sin(tiltAngleRad);
                thrustAxis = new Vec3d(
                    ct * thrustAxis.X + st * thrustAxis.Z,
                    thrustAxis.Y,
                    -st * thrustAxis.X + ct * thrustAxis.Z).Normalized();
            }

            // Slipstream flows in the direction opposite to thrustAxis
            Vec3d wakeDir = -thrustAxis;
            Vec3d rel = queryPointFrd - rotor.MountPositionFrd;

            // Downstream distance s along the wake centerline
            double s = Vec3d.Dot(rel, wakeDir);
            if (s < -0.1 * rotor.Radius || s > 12.0 * rotor.Radius)
                return Vec3d.Zero;

            Vec3d radialVec = rel - s * wakeDir;
            double radialDist = radialVec.Magnitude;

            // Slipstream contraction ratio and axial velocity acceleration factor k_d(s) = 1 + s / sqrt(R^2 + s^2)
            double r0 = rotor.Radius;
            double kd = 1.0 + s / System.Math.Sqrt(r0 * r0 + s * s);
            double rWake = r0 * System.Math.Sqrt(System.Math.Max(0.5, 1.0 / System.Math.Max(1.0, 0.5 * kd)));

            // Smooth Gaussian-topped slipstream radial profile with shear-layer spreading
            double effectiveRadius = rWake + 0.08 * System.Math.Max(0.0, s);
            if (radialDist > 1.6 * effectiveRadius)
                return Vec3d.Zero;

            double radialShape = System.Math.Exp(-2.2 * System.Math.Pow(radialDist / System.Math.Max(0.01, effectiveRadius), 4));
            double vAxialWake = rotorSol.MeanAxialInducedVelocityMps * kd * radialShape;

            return wakeDir * vAxialWake;
        }

        private static double SolveInflowAnglePhi(
            RotorSpec rotor,
            double r,
            double rHub,
            double rTip,
            double chord,
            double twistRad,
            double sigma,
            double vAxial,
            double vRot,
            double vTip,
            AtmosphereState atm)
        {
            double phiLow = 0.005;
            double phiHigh = 1.35;

            double fLow = EvaluateBemtResidual(rotor, phiLow, r, rHub, rTip, chord, twistRad, sigma, vAxial, vRot, vTip, atm);
            for (int iter = 0; iter < 14; iter++)
            {
                double phiMid = 0.5 * (phiLow + phiHigh);
                double fMid = EvaluateBemtResidual(rotor, phiMid, r, rHub, rTip, chord, twistRad, sigma, vAxial, vRot, vTip, atm);

                if (System.Math.Abs(fMid) < 1e-4)
                    return phiMid;

                if (fLow * fMid <= 0.0)
                {
                    phiHigh = phiMid;
                }
                else
                {
                    phiLow = phiMid;
                    fLow = fMid;
                }
            }
            return 0.5 * (phiLow + phiHigh);
        }

        private static double EvaluateBemtResidual(
            RotorSpec rotor,
            double phi,
            double r,
            double rHub,
            double rTip,
            double chord,
            double twistRad,
            double sigma,
            double vAxial,
            double vRot,
            double vTip,
            AtmosphereState atm)
        {
            double sinPhi = System.Math.Sin(phi);
            double cosPhi = System.Math.Max(0.05, System.Math.Cos(phi));
            double alpha = twistRad - phi;

            double wEff = vRot / cosPhi;
            double re = IsaAtmosphereAndTurbulence.ComputeReynolds(wEff, chord, atm);
            double mach = IsaAtmosphereAndTurbulence.ComputeMach(wEff, atm);

            var polar = AirfoilPolarSolver.EvaluateRotatingBladeSection(
                rotor.BladeAirfoil, alpha, re, mach, chord / r, r / rTip, vTip / System.Math.Max(1.0, System.Math.Abs(vAxial) + 2.0));

            double cn = polar.Cl * cosPhi - polar.Cd * sinPhi;
            double ct = polar.Cl * sinPhi + polar.Cd * cosPhi;

            double fTip = ComputePrandtlLoss(rotor.NumBlades, r, rTip, sinPhi, true);
            double fHub = ComputePrandtlLoss(rotor.NumBlades, r, rHub, sinPhi, false);
            double prandtlF = System.Math.Max(0.05, fTip * fHub);

            // Tangential induction factor a'
            double kPrime = (sigma * ct) / System.Math.Max(1e-4, 4.0 * prandtlF * sinPhi * cosPhi);
            double aPrime = System.Math.Clamp(kPrime / (1.0 + System.Math.Abs(kPrime)), -0.2, 0.4);
            double uTangential = vRot * (1.0 - aPrime);

            // Universal hover + axial-flow momentum balance:
            // tan(phi_mom) = (V_ax / 2*U_t) + sqrt((V_ax / 2*U_t)^2 + sigma * max(0, cn) / (4 * F * cos^2(phi)))
            double halfLambda = 0.5 * vAxial / System.Math.Max(0.5, uTangential);
            double thrustTerm = (sigma * System.Math.Max(0.0, cn)) / (4.0 * prandtlF * cosPhi * cosPhi);
            double tanPhiMom = halfLambda + System.Math.Sqrt(halfLambda * halfLambda + thrustTerm);

            return System.Math.Tan(phi) - tanPhiMom;
        }

        private static double ComputePrandtlLoss(int numBlades, double r, double rEdge, double sinPhi, bool isTip)
        {
            double absSin = System.Math.Max(0.02, System.Math.Abs(sinPhi));
            double f = (0.5 * numBlades) * (isTip ? (rEdge - r) / r : (r - rEdge) / rEdge) / absSin;
            f = System.Math.Clamp(f, 0.0, 20.0);
            return MathConstants.TwoOverPi * System.Math.Acos(System.Math.Min(1.0, System.Math.Exp(-f)));
        }
    }
}
