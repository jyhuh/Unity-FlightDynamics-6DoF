// Licensed under the Apache License, Version 2.0.
using System;

namespace FlightDynamics6DoF.Core.Math
{
    /// <summary>
    /// Double-precision 3D vector used across NED (North-East-Down) and FRD (Forward-Right-Down) aerospace frames.
    /// </summary>
    [Serializable]
    public struct Vec3d : IEquatable<Vec3d>
    {
        public double X;
        public double Y;
        public double Z;

        public static readonly Vec3d Zero = new Vec3d(0.0, 0.0, 0.0);
        public static readonly Vec3d UnitX = new Vec3d(1.0, 0.0, 0.0);
        public static readonly Vec3d UnitY = new Vec3d(0.0, 1.0, 0.0);
        public static readonly Vec3d UnitZ = new Vec3d(0.0, 0.0, 1.0);

        public Vec3d(double x, double y, double z)
        {
            X = x;
            Y = y;
            Z = z;
        }

        public double MagnitudeSquared => X * X + Y * Y + Z * Z;
        public double Magnitude => System.Math.Sqrt(X * X + Y * Y + Z * Z);

        public Vec3d Normalized()
        {
            double mag = Magnitude;
            return mag > 1e-12 ? new Vec3d(X / mag, Y / mag, Z / mag) : Zero;
        }

        public bool IsFinite =>
            !double.IsNaN(X) && !double.IsInfinity(X) &&
            !double.IsNaN(Y) && !double.IsInfinity(Y) &&
            !double.IsNaN(Z) && !double.IsInfinity(Z);

        public static double Dot(Vec3d a, Vec3d b) => a.X * b.X + a.Y * b.Y + a.Z * b.Z;

        public static Vec3d Cross(Vec3d a, Vec3d b) => new Vec3d(
            a.Y * b.Z - a.Z * b.Y,
            a.Z * b.X - a.X * b.Z,
            a.X * b.Y - a.Y * b.X);

        public static Vec3d ClampMagnitude(Vec3d v, double maxLen)
        {
            double m = v.Magnitude;
            if (m > maxLen && m > 1e-12)
                return v * (maxLen / m);
            return v;
        }

        public static Vec3d operator +(Vec3d a, Vec3d b) => new Vec3d(a.X + b.X, a.Y + b.Y, a.Z + b.Z);
        public static Vec3d operator -(Vec3d a, Vec3d b) => new Vec3d(a.X - b.X, a.Y - b.Y, a.Z - b.Z);
        public static Vec3d operator -(Vec3d a) => new Vec3d(-a.X, -a.Y, -a.Z);
        public static Vec3d operator *(Vec3d a, double s) => new Vec3d(a.X * s, a.Y * s, a.Z * s);
        public static Vec3d operator *(double s, Vec3d a) => new Vec3d(a.X * s, a.Y * s, a.Z * s);
        public static Vec3d operator /(Vec3d a, double s) => new Vec3d(a.X / s, a.Y / s, a.Z / s);

        public bool Equals(Vec3d other) =>
            System.Math.Abs(X - other.X) < 1e-9 &&
            System.Math.Abs(Y - other.Y) < 1e-9 &&
            System.Math.Abs(Z - other.Z) < 1e-9;

        public override bool Equals(object? obj) => obj is Vec3d other && Equals(other);
        public override int GetHashCode() => HashCode.Combine(X, Y, Z);
        public override string ToString() => $"({X:F4}, {Y:F4}, {Z:F4})";
    }

    /// <summary>
    /// Hamilton unit quaternion (W, X, Y, Z) representing attitude from Body (FRD) to Inertial (NED) frame.
    /// </summary>
    [Serializable]
    public struct Quatd
    {
        public double W;
        public double X;
        public double Y;
        public double Z;

        public static readonly Quatd Identity = new Quatd(1.0, 0.0, 0.0, 0.0);

        public Quatd(double w, double x, double y, double z)
        {
            W = w;
            X = x;
            Y = y;
            Z = z;
        }

        public Quatd Normalized()
        {
            double n = System.Math.Sqrt(W * W + X * X + Y * Y + Z * Z);
            if (n < 1e-12) return Identity;
            return new Quatd(W / n, X / n, Y / n, Z / n);
        }

        public Quatd Conjugate() => new Quatd(W, -X, -Y, -Z);

        public static Quatd operator *(Quatd a, Quatd b) => new Quatd(
            a.W * b.W - a.X * b.X - a.Y * b.Y - a.Z * b.Z,
            a.W * b.X + a.X * b.W + a.Y * b.Z - a.Z * b.Y,
            a.W * b.Y - a.X * b.Z + a.Y * b.W + a.Z * b.X,
            a.W * b.Z + a.X * b.Y - a.Y * b.X + a.Z * b.W);

        /// <summary>
        /// Rotates a vector from the Body (FRD) frame to the Inertial (NED) frame: v_ned = q * v_body * q^-1.
        /// </summary>
        public Vec3d RotateBodyToNed(Vec3d vBody)
        {
            Vec3d qv = new Vec3d(X, Y, Z);
            Vec3d t = 2.0 * Vec3d.Cross(qv, vBody);
            return vBody + W * t + Vec3d.Cross(qv, t);
        }

        /// <summary>
        /// Rotates a vector from the Inertial (NED) frame to the Body (FRD) frame: v_body = q^-1 * v_ned * q.
        /// </summary>
        public Vec3d RotateNedToBody(Vec3d vNed)
        {
            Vec3d qv = new Vec3d(-X, -Y, -Z);
            Vec3d t = 2.0 * Vec3d.Cross(qv, vNed);
            return vNed + W * t + Vec3d.Cross(qv, t);
        }

        /// <summary>
        /// Computes the time derivative dq/dt = 0.5 * q * [0, omega_body].
        /// </summary>
        public Quatd Derivative(Vec3d omegaBody)
        {
            Quatd omegaQ = new Quatd(0.0, omegaBody.X, omegaBody.Y, omegaBody.Z);
            Quatd prod = this * omegaQ;
            return new Quatd(0.5 * prod.W, 0.5 * prod.X, 0.5 * prod.Y, 0.5 * prod.Z);
        }

        public static Quatd AddScaled(Quatd q, Quatd dq, double dt)
        {
            return new Quatd(
                q.W + dq.W * dt,
                q.X + dq.X * dt,
                q.Y + dq.Y * dt,
                q.Z + dq.Z * dt).Normalized();
        }

        /// <summary>
        /// Constructs quaternion from aerospace Euler ZYX angles (roll phi, pitch theta, yaw psi in radians).
        /// </summary>
        public static Quatd FromEulerZyx(double rollRad, double pitchRad, double yawRad)
        {
            double cr = System.Math.Cos(rollRad * 0.5);
            double sr = System.Math.Sin(rollRad * 0.5);
            double cp = System.Math.Cos(pitchRad * 0.5);
            double sp = System.Math.Sin(pitchRad * 0.5);
            double cy = System.Math.Cos(yawRad * 0.5);
            double sy = System.Math.Sin(yawRad * 0.5);

            return new Quatd(
                cr * cp * cy + sr * sp * sy,
                sr * cp * cy - cr * sp * sy,
                cr * sp * cy + sr * cp * sy,
                cr * cp * sy - sr * sp * cy).Normalized();
        }

        /// <summary>
        /// Converts quaternion to aerospace Euler ZYX angles (Roll, Pitch, Yaw in radians).
        /// </summary>
        public Vec3d ToEulerZyx()
        {
            double sinrCosp = 2.0 * (W * X + Y * Z);
            double cosrCosp = 1.0 - 2.0 * (X * X + Y * Y);
            double roll = System.Math.Atan2(sinrCosp, cosrCosp);

            double sinp = System.Math.Clamp(2.0 * (W * Y - Z * X), -1.0, 1.0);
            double pitch = System.Math.Asin(sinp);

            double sinyCosp = 2.0 * (W * Z + X * Y);
            double cosyCosp = 1.0 - 2.0 * (Y * Y + Z * Z);
            double yaw = System.Math.Atan2(sinyCosp, cosyCosp);

            return new Vec3d(roll, pitch, yaw);
        }

        /// <summary>
        /// Computes SO(3)/Quaternion attitude error vector in body frame from current attitude to desired attitude.
        /// </summary>
        public static Vec3d AttitudeErrorBody(Quatd current, Quatd desired)
        {
            Quatd qErr = current.Conjugate() * desired;
            double sign = qErr.W >= 0.0 ? 1.0 : -1.0;
            return new Vec3d(2.0 * sign * qErr.X, 2.0 * sign * qErr.Y, 2.0 * sign * qErr.Z);
        }
    }

    /// <summary>
    /// 3x3 double-precision matrix for inertia tensors and linear transformations.
    /// </summary>
    [Serializable]
    public struct Mat3d
    {
        public double M00, M01, M02;
        public double M10, M11, M12;
        public double M20, M21, M22;

        public Mat3d(
            double m00, double m01, double m02,
            double m10, double m11, double m12,
            double m20, double m21, double m22)
        {
            M00 = m00; M01 = m01; M02 = m02;
            M10 = m10; M11 = m11; M12 = m12;
            M20 = m20; M21 = m21; M22 = m22;
        }

        public static Mat3d Diagonal(double ixx, double iyy, double izz) =>
            new Mat3d(ixx, 0, 0, 0, iyy, 0, 0, 0, izz);

        public static Mat3d FromInertiaTensor(double ixx, double iyy, double izz, double ixy = 0, double ixz = 0, double iyz = 0) =>
            new Mat3d(
                 ixx, -ixy, -ixz,
                -ixy,  iyy, -iyz,
                -ixz, -iyz,  izz);

        public Vec3d Multiply(Vec3d v) => new Vec3d(
            M00 * v.X + M01 * v.Y + M02 * v.Z,
            M10 * v.X + M11 * v.Y + M12 * v.Z,
            M20 * v.X + M21 * v.Y + M22 * v.Z);

        public double Determinant =>
            M00 * (M11 * M22 - M12 * M21) -
            M01 * (M10 * M22 - M12 * M20) +
            M02 * (M10 * M21 - M11 * M20);

        public Mat3d Inverse()
        {
            double det = Determinant;
            if (System.Math.Abs(det) < 1e-14)
                return Diagonal(1.0 / System.Math.Max(M00, 1e-6), 1.0 / System.Math.Max(M11, 1e-6), 1.0 / System.Math.Max(M22, 1e-6));

            double invDet = 1.0 / det;
            return new Mat3d(
                (M11 * M22 - M12 * M21) * invDet,
                (M02 * M21 - M01 * M22) * invDet,
                (M01 * M12 - M02 * M11) * invDet,
                (M12 * M20 - M10 * M22) * invDet,
                (M00 * M22 - M02 * M20) * invDet,
                (M02 * M10 - M00 * M12) * invDet,
                (M10 * M21 - M11 * M20) * invDet,
                (M01 * M20 - M00 * M21) * invDet,
                (M00 * M11 - M01 * M10) * invDet);
        }
    }

    /// <summary>
    /// Dense linear algebra utilities for VLM aerodynamic influence matrices and Pseudoinverse Control Allocation.
    /// </summary>
    public static class LinearAlgebra
    {
        public static double[] SolveLinearSystem(double[,] A, double[] b)
        {
            int n = b.Length;
            double[,] aug = new double[n, n + 1];
            for (int i = 0; i < n; i++)
            {
                for (int j = 0; j < n; j++) aug[i, j] = A[i, j];
                aug[i, n] = b[i];
            }

            for (int col = 0; col < n; col++)
            {
                int pivot = col;
                double maxVal = System.Math.Abs(aug[col, col]);
                for (int row = col + 1; row < n; row++)
                {
                    double val = System.Math.Abs(aug[row, col]);
                    if (val > maxVal)
                    {
                        maxVal = val;
                        pivot = row;
                    }
                }

                if (pivot != col)
                {
                    for (int j = col; j <= n; j++)
                    {
                        double tmp = aug[col, j];
                        aug[col, j] = aug[pivot, j];
                        aug[pivot, j] = tmp;
                    }
                }

                double diag = aug[col, col];
                if (System.Math.Abs(diag) < 1e-12)
                    diag = diag >= 0 ? 1e-12 : -1e-12;

                for (int row = col + 1; row < n; row++)
                {
                    double factor = aug[row, col] / diag;
                    for (int j = col; j <= n; j++)
                        aug[row, j] -= factor * aug[col, j];
                }
            }

            double[] x = new double[n];
            for (int i = n - 1; i >= 0; i--)
            {
                double sum = aug[i, n];
                for (int j = i + 1; j < n; j++)
                    sum -= aug[i, j] * x[j];
                double diag = aug[i, i];
                if (System.Math.Abs(diag) < 1e-12) diag = 1e-12;
                x[i] = sum / diag;
            }
            return x;
        }

        /// <summary>
        /// Computes damped right Moore-Penrose pseudoinverse A^+ = A^T (A A^T + lambda^2 I)^-1 for m x n matrix (m &lt;= n).
        /// </summary>
        public static double[,] ComputeDampedRightPseudoinverse(double[,] A, double damping = 1e-6)
        {
            int m = A.GetLength(0);
            int n = A.GetLength(1);
            double[,] aat = new double[m, m];
            for (int i = 0; i < m; i++)
            {
                for (int j = 0; j < m; j++)
                {
                    double s = 0.0;
                    for (int k = 0; k < n; k++) s += A[i, k] * A[j, k];
                    if (i == j) s += damping * damping;
                    aat[i, j] = s;
                }
            }

            // Invert m x m matrix aat
            double[,] aatInv = new double[m, m];
            for (int col = 0; col < m; col++)
            {
                double[] e = new double[m];
                e[col] = 1.0;
                double[] sol = SolveLinearSystem(aat, e);
                for (int row = 0; row < m; row++)
                    aatInv[row, col] = sol[row];
            }

            // A^+ = A^T * aatInv (n x m)
            double[,] pinv = new double[n, m];
            for (int i = 0; i < n; i++)
            {
                for (int j = 0; j < m; j++)
                {
                    double s = 0.0;
                    for (int k = 0; k < m; k++)
                        s += A[k, i] * aatInv[k, j];
                    pinv[i, j] = s;
                }
            }
            return pinv;
        }
    }

    /// <summary>
    /// Converts between Aerospace Standard Right-Handed (NED / FRD) frames,
    /// Unity Left-Handed (EUN / RUF: X=East/Right, Y=Up, Z=North/Forward) frames,
    /// and WGS84 Geodetic coordinates.
    /// </summary>
    public static class FrameConverter
    {
        private const double Wgs84A = Wgs84Constants.SemiMajorAxisM;
        private const double Wgs84E2 = Wgs84Constants.EccentricitySquaredE2;
        public const double Deg2Rad = MathConstants.Deg2Rad;
        public const double Rad2Deg = MathConstants.Rad2Deg;

        /// <summary>
        /// Converts NED position (North, East, Down) [m] to Unity world position (X=East, Y=Up=-Down, Z=North) [m].
        /// </summary>
        public static Vec3d NedToUnityPosition(Vec3d ned) => new Vec3d(ned.Y, -ned.Z, ned.X);

        /// <summary>
        /// Converts Unity world position (X=East, Y=Up, Z=North) [m] to NED position (North, East, Down) [m].
        /// </summary>
        public static Vec3d UnityToNedPosition(Vec3d unityPos) => new Vec3d(unityPos.Z, unityPos.X, -unityPos.Y);

        /// <summary>
        /// Converts FRD body vector (Forward, Right, Down) to Unity local body vector (X=Right, Y=Up, Z=Forward).
        /// </summary>
        public static Vec3d FrdToUnityVector(Vec3d frd) => new Vec3d(frd.Y, -frd.Z, frd.X);

        /// <summary>
        /// Converts Unity local body vector (X=Right, Y=Up, Z=Forward) to FRD body vector (Forward, Right, Down).
        /// </summary>
        public static Vec3d UnityToFrdVector(Vec3d unityLocal) => new Vec3d(unityLocal.Z, unityLocal.X, -unityLocal.Y);

        /// <summary>
        /// Converts FRD angular velocity / torque (Roll-X, Pitch-Y, Yaw-Z right-handed) to Unity left-handed torque.
        /// Due to handedness flip alongside axis permutation, torque in Unity is (-Pitch_FRD, Yaw_FRD, -Roll_FRD).
        /// </summary>
        public static Vec3d FrdToUnityTorque(Vec3d frdTorque) => new Vec3d(-frdTorque.Y, frdTorque.Z, -frdTorque.X);

        /// <summary>
        /// Converts Aerospace NED/FRD quaternion to Unity Left-Handed quaternion (X, Y, Z, W).
        /// </summary>
        public static Quatd NedToUnityRotation(Quatd qNed) => new Quatd(qNed.W, -qNed.Y, qNed.Z, -qNed.X).Normalized();

        /// <summary>
        /// Converts Unity Left-Handed rotation quaternion to Aerospace NED/FRD quaternion.
        /// </summary>
        public static Quatd UnityToNedRotation(Quatd qUnity) => new Quatd(qUnity.W, -qUnity.Z, -qUnity.X, qUnity.Y).Normalized();

        /// <summary>
        /// Converts local NED offset [m] from a reference WGS84 origin (lat0, lon0, alt0) to WGS84 (Lat [deg], Lon [deg], Alt MSL [m]).
        /// </summary>
        public static Vec3d NedToWgs84(Vec3d nedMeters, double originLatDeg, double originLonDeg, double originAltMsl)
        {
            double lat0 = originLatDeg * Deg2Rad;
            double sinLat = System.Math.Sin(lat0);
            double cosLat = System.Math.Max(1e-9, System.Math.Cos(lat0));
            double denom = System.Math.Sqrt(1.0 - Wgs84E2 * sinLat * sinLat);
            double rN = Wgs84A * (1.0 - Wgs84E2) / (denom * denom * denom);
            double rE = Wgs84A / denom;

            double dLat = nedMeters.X / (rN + originAltMsl);
            double dLon = nedMeters.Y / ((rE + originAltMsl) * cosLat);
            double alt = originAltMsl - nedMeters.Z;

            return new Vec3d(
                originLatDeg + dLat * Rad2Deg,
                originLonDeg + dLon * Rad2Deg,
                alt);
        }

        /// <summary>
        /// Converts WGS84 (Lat [deg], Lon [deg], Alt MSL [m]) to local NED position [m] relative to reference origin.
        /// </summary>
        public static Vec3d Wgs84ToNed(double latDeg, double lonDeg, double altMsl, double originLatDeg, double originLonDeg, double originAltMsl)
        {
            double lat0 = originLatDeg * Deg2Rad;
            double sinLat = System.Math.Sin(lat0);
            double cosLat = System.Math.Max(1e-9, System.Math.Cos(lat0));
            double denom = System.Math.Sqrt(1.0 - Wgs84E2 * sinLat * sinLat);
            double rN = Wgs84A * (1.0 - Wgs84E2) / (denom * denom * denom);
            double rE = Wgs84A / denom;

            double north = (latDeg - originLatDeg) * Deg2Rad * (rN + originAltMsl);
            double east = (lonDeg - originLonDeg) * Deg2Rad * (rE + originAltMsl) * cosLat;
            double down = -(altMsl - originAltMsl);

            return new Vec3d(north, east, down);
        }
    }
}
