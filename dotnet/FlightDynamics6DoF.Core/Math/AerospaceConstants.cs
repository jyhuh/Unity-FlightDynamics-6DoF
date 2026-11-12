// Licensed under the Apache License, Version 2.0.
using System;

namespace FlightDynamics6DoF.Core.Math
{
    /// <summary>
    /// Mathematical and angular conversion constants.
    /// </summary>
    public readonly struct MathConstants
    {
        public const double Deg2Rad = System.Math.PI / 180.0;
        public const double Rad2Deg = 180.0 / System.Math.PI;
        public const double TwoPi = 2.0 * System.Math.PI;
        public const double HalfPi = System.Math.PI * 0.5;
        public const double InvPi = 1.0 / System.Math.PI;
        public const double TwoOverPi = 2.0 / System.Math.PI;
        public const double InvTwoPi = 1.0 / (2.0 * System.Math.PI);
        public const double InvFourPi = 1.0 / (4.0 * System.Math.PI);
        public const double RpmToRadPerSec = TwoPi / 60.0;
        public const double RadPerSecToRpm = 60.0 / TwoPi;
    }

    /// <summary>
    /// Standard physical and atmospheric constants based on the International Standard Atmosphere (ISA).
    /// </summary>
    public readonly struct PhysicalConstants
    {
        public const double GravityMps2 = 9.80665;
        public const double SeaLevelTempK = 288.15;
        public const double SeaLevelPressurePa = 101325.0;
        public const double SeaLevelDensityKgM3 = 1.225;
        public const double LapseRateKPerM = 0.0065;
        public const double GasConstantR = 287.05287;
        public const double GammaAir = 1.4;

        // Sutherland's Viscosity Law Constants for Air
        public const double SutherlandConstantS = 110.4;
        public const double SutherlandRefViscosityMu0 = 1.7894e-5;
        public const double SutherlandRefTempT0 = 288.15;
    }

    /// <summary>
    /// World Geodetic System 1984 (WGS-84) Earth reference ellipsoid constants.
    /// </summary>
    public readonly struct Wgs84Constants
    {
        public const double SemiMajorAxisM = 6378137.0;
        public const double FlatteningF = 1.0 / 298.257223563;
        public const double EccentricitySquaredE2 = FlatteningF * (2.0 - FlatteningF);
    }

    /// <summary>
    /// Runway ground contact, landing gear friction, and surface boundary layer constants.
    /// </summary>
    public readonly struct GroundContactConstants
    {
        /// <summary>
        /// Coulomb rolling friction coefficient on paved asphalt/concrete runway (~0.025).
        /// </summary>
        public const double PavedRunwayRollingFrictionMu = 0.025;

        /// <summary>
        /// Coulomb lateral tire cornering friction limit (~0.70).
        /// </summary>
        public const double LateralTireFrictionMu = 0.70;

        /// <summary>
        /// Compliant ground contact spring stiffness multiplier per unit vehicle mass (120 N/m per kg).
        /// </summary>
        public const double SpringStiffnessPerKg = 120.0;

        /// <summary>
        /// Compliant ground contact vertical damper multiplier per unit vehicle mass (22 N/(m/s) per kg).
        /// </summary>
        public const double DamperRatePerKg = 22.0;

        /// <summary>
        /// Throttle threshold below which parking brake / resting ground hold is engaged (&lt; 5%).
        /// </summary>
        public const double ParkingBrakeThrottleThreshold = 0.05;

        /// <summary>
        /// Pitch floor angle in aerospace FRD coordinates preventing nose-gear ground penetration (0.015 rad).
        /// </summary>
        public const double NoseGearPitchDownLimitRad = 0.015;

        /// <summary>
        /// Maximum allowable bank/roll angle in radians while landing gear wheels are on the runway (~3.0 deg = 0.052 rad).
        /// Prevents wingtip strike and lateral ground roll-over.
        /// </summary>
        public const double WingtipStrikeRollLimitRad = 0.052;

        /// <summary>
        /// Landing gear track-width roll restoring stiffness per unit vehicle mass (35.0 N*m/rad per kg).
        /// Resists motor reaction torque and prevents lateral tip-over on runway.
        /// </summary>
        public const double GearRollRestoringStiffnessPerKg = 35.0;

        /// <summary>
        /// Landing gear roll damping rate per unit vehicle mass (6.0 N*m/(rad/s) per kg).
        /// </summary>
        public const double GearRollDampingRatePerKg = 6.0;

        /// <summary>
        /// Nose-gear pitch-up restoring stiffness per unit vehicle mass (25.0 N*m/rad per kg).
        /// </summary>
        public const double NoseGearPitchRestoringStiffnessPerKg = 25.0;

        /// <summary>
        /// Nose-gear pitch damping rate per unit vehicle mass (4.0 N*m/(rad/s) per kg).
        /// </summary>
        public const double NoseGearPitchDampingRatePerKg = 4.0;

        /// <summary>
        /// Surface atmospheric boundary layer height over which wind gusts ramp from zero to full intensity (1.5 m).
        /// </summary>
        public const double BoundaryLayerHeightMeters = 1.5;

        /// <summary>
        /// Height AGL below which gust turbulence is quenched to zero (0.05 m).
        /// </summary>
        public const double BoundaryLayerFloorMeters = 0.05;
    }

    /// <summary>
    /// Standard network ports and protocols for PX4 SITL, QGroundControl, and ROS 2 communication.
    /// </summary>
    public readonly struct NetworkConstants
    {
        public const int Px4SitlTcpPort = 4560;
        public const int QgcTelemetryUdpPort = 14550;
        public const int LocalAutopilotUdpPort = 14540;
        public const int MicroXrceDdsUdpPort = 8888;
    }
}
