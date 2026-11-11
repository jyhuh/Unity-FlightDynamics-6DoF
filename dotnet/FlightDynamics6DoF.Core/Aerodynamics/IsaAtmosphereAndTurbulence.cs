// Licensed under the Apache License, Version 2.0.
using System;
using FlightDynamics6DoF.Core.Math;

namespace FlightDynamics6DoF.Core.Aerodynamics
{
    public struct AtmosphereState
    {
        public double AltitudeMsl;
        public double TemperatureKelvin;
        public double PressurePascal;
        public double DensityKgPerM3;
        public double SpeedOfSoundMps;
        public double DynamicViscosityPaS;
        public double KinematicViscosityM2ps;
    }

    /// <summary>
    /// International Standard Atmosphere (ISA) calculator and 3-axis continuous Dryden Wind Turbulence generator.
    /// </summary>
    public class IsaAtmosphereAndTurbulence
    {
        public const double SeaLevelTempK = 288.15;
        public const double SeaLevelPressurePa = 101325.0;
        public const double SeaLevelDensity = 1.225;
        public const double LapseRateKPerM = 0.0065;
        public const double GasConstantR = 287.05287;
        public const double GammaAir = 1.4;
        public const double GravityMps2 = 9.80665;

        private readonly Random _rng;
        private Vec3d _turbulenceStateBody;

        public IsaAtmosphereAndTurbulence(int seed = 1337)
        {
            _rng = new Random(seed);
            _turbulenceStateBody = Vec3d.Zero;
        }

        public static AtmosphereState Evaluate(double altitudeMslMeters)
        {
            double h = System.Math.Max(-500.0, System.Math.Min(altitudeMslMeters, 20000.0));
            double tempK;
            double pressurePa;

            if (h <= 11000.0)
            {
                tempK = SeaLevelTempK - LapseRateKPerM * h;
                pressurePa = SeaLevelPressurePa * System.Math.Pow(tempK / SeaLevelTempK, GravityMps2 / (LapseRateKPerM * GasConstantR));
            }
            else
            {
                tempK = 216.65;
                double p11 = SeaLevelPressurePa * System.Math.Pow(216.65 / SeaLevelTempK, GravityMps2 / (LapseRateKPerM * GasConstantR));
                pressurePa = p11 * System.Math.Exp(-GravityMps2 * (h - 11000.0) / (GasConstantR * tempK));
            }

            double rho = pressurePa / (GasConstantR * tempK);
            double a = System.Math.Sqrt(GammaAir * GasConstantR * tempK);

            // Sutherland's formula for dynamic viscosity
            double mu = 1.458e-6 * System.Math.Pow(tempK, 1.5) / (tempK + 110.4);

            return new AtmosphereState
            {
                AltitudeMsl = h,
                TemperatureKelvin = tempK,
                PressurePascal = pressurePa,
                DensityKgPerM3 = rho,
                SpeedOfSoundMps = a,
                DynamicViscosityPaS = mu,
                KinematicViscosityM2ps = mu / rho
            };
        }

        public static double ComputeReynolds(double airspeedMps, double characteristicLengthM, AtmosphereState atm)
        {
            double v = System.Math.Max(0.5, airspeedMps);
            double l = System.Math.Max(0.005, characteristicLengthM);
            double re = (atm.DensityKgPerM3 * v * l) / atm.DynamicViscosityPaS;
            // Clamp to specification operational range [10^4, 10^7]
            return System.Math.Max(1.0e4, System.Math.Min(re, 1.0e7));
        }

        public static double ComputeMach(double airspeedMps, AtmosphereState atm)
        {
            double m = System.Math.Abs(airspeedMps) / System.Math.Max(100.0, atm.SpeedOfSoundMps);
            // Subsonic validity envelope M < 0.7
            return System.Math.Min(0.699, m);
        }

        /// <summary>
        /// Steps the 3-axis continuous Dryden turbulence filter (MIL-F-8785C low-altitude scale lengths)
        /// and returns the gust velocity vector in the Body (FRD) frame [m/s].
        /// </summary>
        public Vec3d StepDrydenTurbulenceBody(double dt, double airspeedMps, double altitudeAglMeters, double turbulenceSigmaMps)
        {
            if (turbulenceSigmaMps <= 1e-4 || dt <= 0.0)
                return Vec3d.Zero;

            // When resting on or near the ground, atmospheric surface boundary layer heavily dampens gust turbulence
            double groundAtten = System.Math.Max(0.0, System.Math.Min(1.0, (altitudeAglMeters - 0.05) / 1.5));
            if (groundAtten <= 0.001)
            {
                _turbulenceStateBody = Vec3d.Zero;
                return Vec3d.Zero;
            }

            double h = System.Math.Max(3.0, altitudeAglMeters);
            double v = System.Math.Max(3.0, airspeedMps);

            // Low-altitude Dryden scale lengths L_u, L_v, L_w
            double lw = h;
            double lu = h / System.Math.Pow(0.177 + 0.0027 * h, 1.2);
            double lv = lu;

            double sigmaW = turbulenceSigmaMps * groundAtten;
            double sigmaU = sigmaW / System.Math.Pow(0.177 + 0.0027 * h, 0.4);
            double sigmaV = sigmaU;

            double tauU = System.Math.Max(0.05, lu / v);
            double tauV = System.Math.Max(0.05, lv / v);
            double tauW = System.Math.Max(0.05, lw / v);

            double alphaU = System.Math.Exp(-dt / tauU);
            double alphaV = System.Math.Exp(-dt / tauV);
            double alphaW = System.Math.Exp(-dt / tauW);

            double nx = NextGaussian();
            double ny = NextGaussian();
            double nz = NextGaussian();

            _turbulenceStateBody.X = alphaU * _turbulenceStateBody.X + sigmaU * System.Math.Sqrt(1.0 - alphaU * alphaU) * nx;
            _turbulenceStateBody.Y = alphaV * _turbulenceStateBody.Y + sigmaV * System.Math.Sqrt(1.0 - alphaV * alphaV) * ny;
            _turbulenceStateBody.Z = alphaW * _turbulenceStateBody.Z + sigmaW * System.Math.Sqrt(1.0 - alphaW * alphaW) * nz;

            return _turbulenceStateBody;
        }

        private double NextGaussian()
        {
            double u1 = 1.0 - _rng.NextDouble();
            double u2 = 1.0 - _rng.NextDouble();
            return System.Math.Sqrt(-2.0 * System.Math.Log(u1)) * System.Math.Cos(2.0 * System.Math.PI * u2);
        }
    }
}
