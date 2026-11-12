// Licensed under the Apache License, Version 2.0.
using System;
using FlightDynamics6DoF.Core.Aerodynamics;
using FlightDynamics6DoF.Core.Dynamics;
using FlightDynamics6DoF.Core.Math;
using FlightDynamics6DoF.Core.Specs;

namespace FlightDynamics6DoF.Core.Sensors
{
    public struct VirtualSensorReadings
    {
        public ulong TimestampMicroseconds;
        public Vec3d ImuAccelBodyMps2;
        public Vec3d ImuGyroBodyRadps;
        public Vec3d MagnetometerBodyGauss;
        public double BaroPressureHpa;
        public double BaroAltitudeMsl;
        public double TemperatureCelsius;
        public double DifferentialPressureHpa;
        public double RangefinderDistanceMeters;

        public bool GpsUpdatedThisStep;
        public double GpsLatitudeDeg;
        public double GpsLongitudeDeg;
        public double GpsAltitudeMsl;
        public Vec3d GpsVelocityNedMps;
        public double GpsGroundSpeedMps;
        public double GpsCourseOverGroundDeg;
        public int GpsFixType;
        public int GpsSatellitesVisible;
    }

    /// <summary>
    /// Simulates aerospace sensors (IMU Specific Force + Gyro with Gaussian White Noise &amp; Bias Random Walk,
    /// WGS84 GPS, ISA Barometer, 3-axis Magnetometer, and AGL Rangefinder) feeding PX4/ArduPilot SITL and Internal GNC.
    /// </summary>
    public class VirtualSensorSuite
    {
        private readonly SensorSuiteSpec _spec;
        private readonly Random _rng;
        private Vec3d _accelBias;
        private Vec3d _gyroBias;
        private double _lastGpsUpdateTimeSec = -1.0;
        private VirtualSensorReadings _lastReadings;

        public VirtualSensorSuite(SensorSuiteSpec spec)
        {
            _spec = spec ?? new SensorSuiteSpec();
            _rng = new Random(_spec.RandomSeed);
            _accelBias = Vec3d.Zero;
            _gyroBias = Vec3d.Zero;
        }

        public VirtualSensorReadings Sample(
            SixDofState state,
            double dt,
            double originLatDeg,
            double originLonDeg,
            double originAltMsl)
        {
            double sqrtDt = System.Math.Sqrt(System.Math.Max(1e-5, dt));

            // 1. Update IMU Bias Random Walk
            _accelBias += RandomGaussianVec3() * (_spec.AccelBiasRandomWalk * sqrtDt);
            _gyroBias += RandomGaussianVec3() * (_spec.GyroBiasRandomWalk * sqrtDt);

            // Specific force at IMU mount offset: f_imu = f_cg + alpha x r + omega x (omega x r)
            Vec3d rImu = _spec.ImuMountOffsetFrd;
            Vec3d leverArmAccel = Vec3d.Cross(state.AngularAccelerationBodyFrd, rImu) +
                                  Vec3d.Cross(state.AngularVelocityBodyFrd, Vec3d.Cross(state.AngularVelocityBodyFrd, rImu));

            Vec3d imuAccel = state.SpecificForceBodyFrd + leverArmAccel + _accelBias + RandomGaussianVec3() * _spec.AccelNoiseStdMps2;
            Vec3d imuGyro = state.AngularVelocityBodyFrd + _gyroBias + RandomGaussianVec3() * _spec.GyroNoiseStdRadps;

            // 2. Magnetometer (Earth field rotated from NED to Body FRD + noise)
            Vec3d magBody = state.AttitudeBodyToNed.RotateNedToBody(_spec.MagEarthFieldGaussNed) +
                            RandomGaussianVec3() * _spec.MagNoiseStdGauss;

            // 3. Barometer (ISA Static Pressure at true MSL altitude + noise)
            double trueAltMsl = originAltMsl - state.PositionNed.Z;
            var atm = IsaAtmosphereAndTurbulence.Evaluate(trueAltMsl);
            double pressureHpa = (atm.PressurePascal / 100.0) + NextGaussian() * _spec.BaroNoiseStdHpa;
            double baroAltMsl = 44330.0 * (1.0 - System.Math.Pow(pressureHpa / 1013.25, 0.19029495));

            // 4. Pitot Tube / Differential Pressure for Airspeed Sensor (hPa = 100 Pa)
            // q = 0.5 * rho * V_ias^2. In hPa: q_pa / 100.0
            double ias = System.Math.Max(0.0, state.AirspeedMps);
            double diffPressureHpa = (0.5 * atm.DensityKgPerM3 * ias * ias) / 100.0;
            if (diffPressureHpa > 0.0)
            {
                // Add subtle sensor noise (approx 0.02 hPa RMS)
                diffPressureHpa = System.Math.Max(0.0, diffPressureHpa + NextGaussian() * 0.02);
            }

            // 5. Rangefinder (AGL distance along body +Z_FRD ray)
            Vec3d bodyDownNed = state.AttitudeBodyToNed.RotateBodyToNed(Vec3d.UnitZ);
            double cosTilt = System.Math.Max(0.15, bodyDownNed.Z);
            double slantRange = System.Math.Min(_spec.RangefinderMaxRangeMeters, state.AltitudeAglMeters / cosTilt + NextGaussian() * _spec.RangefinderNoiseStdMeters);

            // 5. GPS update at configured rate (default 10 Hz)
            double gpsPeriod = 1.0 / System.Math.Max(1.0, _spec.GpsUpdateRateHz);
            bool gpsUpdated = false;
            if (_lastGpsUpdateTimeSec < 0.0 || (state.SimTimeSec - _lastGpsUpdateTimeSec) >= gpsPeriod)
            {
                _lastGpsUpdateTimeSec = state.SimTimeSec;
                gpsUpdated = true;

                Vec3d noisyPosNed = state.PositionNed + new Vec3d(
                    NextGaussian() * _spec.GpsHorizontalNoiseStdMeters,
                    NextGaussian() * _spec.GpsHorizontalNoiseStdMeters,
                    NextGaussian() * _spec.GpsVerticalNoiseStdMeters);

                Vec3d lla = FrameConverter.NedToWgs84(noisyPosNed, originLatDeg, originLonDeg, originAltMsl);
                Vec3d noisyVelNed = state.VelocityNed + RandomGaussianVec3() * _spec.GpsVelocityNoiseStdMps;

                double groundSpeed = System.Math.Sqrt(noisyVelNed.X * noisyVelNed.X + noisyVelNed.Y * noisyVelNed.Y);
                double cogDeg = System.Math.Atan2(noisyVelNed.Y, noisyVelNed.X) * FrameConverter.Rad2Deg;
                if (cogDeg < 0.0) cogDeg += 360.0;

                _lastReadings.GpsLatitudeDeg = lla.X;
                _lastReadings.GpsLongitudeDeg = lla.Y;
                _lastReadings.GpsAltitudeMsl = lla.Z;
                _lastReadings.GpsVelocityNedMps = noisyVelNed;
                _lastReadings.GpsGroundSpeedMps = groundSpeed;
                _lastReadings.GpsCourseOverGroundDeg = cogDeg;
                _lastReadings.GpsFixType = 3; // 3D Fix
                _lastReadings.GpsSatellitesVisible = 14;
            }

            _lastReadings.TimestampMicroseconds = 1_000_000UL + (ulong)(state.SimTimeSec * 1.0e6);
            _lastReadings.ImuAccelBodyMps2 = imuAccel;
            _lastReadings.ImuGyroBodyRadps = imuGyro;
            _lastReadings.MagnetometerBodyGauss = magBody;
            _lastReadings.BaroPressureHpa = pressureHpa;
            _lastReadings.BaroAltitudeMsl = baroAltMsl;
            _lastReadings.TemperatureCelsius = atm.TemperatureKelvin - 273.15;
            _lastReadings.DifferentialPressureHpa = diffPressureHpa;
            _lastReadings.RangefinderDistanceMeters = System.Math.Max(0.02, slantRange);
            _lastReadings.GpsUpdatedThisStep = gpsUpdated;

            return _lastReadings;
        }

        private Vec3d RandomGaussianVec3() => new Vec3d(NextGaussian(), NextGaussian(), NextGaussian());

        private double NextGaussian()
        {
            double u1 = 1.0 - _rng.NextDouble();
            double u2 = 1.0 - _rng.NextDouble();
            return System.Math.Sqrt(-2.0 * System.Math.Log(u1)) * System.Math.Cos(MathConstants.TwoPi * u2);
        }
    }
}
