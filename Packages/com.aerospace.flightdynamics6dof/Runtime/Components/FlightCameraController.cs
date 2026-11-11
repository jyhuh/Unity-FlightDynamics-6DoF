// Licensed under the Apache License, Version 2.0.
#nullable enable
using UnityEngine;

namespace FlightDynamics6DoF.Unity
{
    public enum CameraViewMode
    {
        ChaseOrbit = 0,
        CloseFollow = 1,
        FpvNoseCam = 2,
        TopDownMap = 3
    }

    /// <summary>
    /// Smooth engineering chase, orbit, FPV, and top-down camera controller that tracks a <see cref="FlightDynamicsVehicle"/>.
    /// Automatically attaches to Main Camera when a simulation scene is initialized.
    /// </summary>
    public class FlightCameraController : MonoBehaviour
    {
        public FlightDynamicsVehicle? TargetVehicle;
        public CameraViewMode ViewMode = CameraViewMode.ChaseOrbit;

        public float Distance = 3.2f;
        public float HeightOffset = 1.15f;
        public float OrbitAzimuthDeg = -25.0f;
        public float OrbitElevationDeg = 20.0f;
        public float FollowSmoothTime = 0.08f;

        private Vector3 _currentVelocity;

        private void LateUpdate()
        {
            if (TargetVehicle == null)
            {
                TargetVehicle = FindFirstObjectByType<FlightDynamicsVehicle>();
                if (TargetVehicle == null) return;
            }

            Transform target = TargetVehicle.transform;

            switch (ViewMode)
            {
                case CameraViewMode.ChaseOrbit:
                {
                    float yawDeg = target.eulerAngles.y + OrbitAzimuthDeg;
                    Quaternion rot = Quaternion.Euler(OrbitElevationDeg, yawDeg, 0f);
                    Vector3 desiredPos = target.position + rot * new Vector3(0f, 0f, -Distance) + Vector3.up * (HeightOffset * 0.3f);
                    transform.position = Vector3.SmoothDamp(transform.position, desiredPos, ref _currentVelocity, FollowSmoothTime);
                    transform.LookAt(target.position + Vector3.up * 0.15f);
                    break;
                }

                case CameraViewMode.CloseFollow:
                {
                    Vector3 backDir = -Vector3.ProjectOnPlane(target.forward, Vector3.up).normalized;
                    if (backDir.sqrMagnitude < 0.01f) backDir = Vector3.back;
                    Vector3 desiredPos = target.position + backDir * (Distance * 0.75f) + Vector3.up * (HeightOffset * 0.65f);
                    transform.position = Vector3.SmoothDamp(transform.position, desiredPos, ref _currentVelocity, FollowSmoothTime);
                    transform.LookAt(target.position + target.forward * 0.8f);
                    break;
                }

                case CameraViewMode.FpvNoseCam:
                {
                    transform.position = target.TransformPoint(new Vector3(0f, 0.06f, 0.22f));
                    transform.rotation = target.rotation;
                    break;
                }

                case CameraViewMode.TopDownMap:
                {
                    Vector3 desiredPos = target.position + new Vector3(0f, 18.0f, -0.5f);
                    transform.position = Vector3.SmoothDamp(transform.position, desiredPos, ref _currentVelocity, FollowSmoothTime);
                    transform.LookAt(target.position);
                    break;
                }
            }
        }
    }
}
