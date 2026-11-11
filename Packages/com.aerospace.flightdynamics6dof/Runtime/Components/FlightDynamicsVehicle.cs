// Licensed under the Apache License, Version 2.0.
#nullable enable
using System;
using System.Collections.Generic;
using UnityEngine;
using FlightDynamics6DoF.Core.Aerodynamics;
using FlightDynamics6DoF.Core.Dynamics;
using FlightDynamics6DoF.Core.Gnc;
using FlightDynamics6DoF.Core.Math;
using FlightDynamics6DoF.Core.Sensors;
using FlightDynamics6DoF.Core.Specs;

namespace FlightDynamics6DoF.Unity
{
    /// <summary>
    /// Primary Unity MonoBehaviour representing an engineered UAV / VTOL / Multirotor vehicle.
    /// Separates Stage 1 (Pre-Simulation Geometry &amp; Aerodynamic Analysis) from Stage 2 (Real-Time Simulation)
    /// and allows runtime switching across Fidelity Levels 0 (Unity Rigidbody) through Level 3 (Full 6-DOF RK4 + 3D VLM + Slipstream).
    /// </summary>
    [DisallowMultipleComponent]
    public class FlightDynamicsVehicle : MonoBehaviour
    {
        [Header("Aircraft Profile & Stage 1 Baked Database")]
        public AircraftProfileAsset? ProfileAsset;
        public AircraftConfigSpec InlineConfig = AircraftConfigSpec.CreateStandardQuadcopterX();
        public BakedAeroDatabase InlineBakedDatabase = new BakedAeroDatabase();

        [Header("Stage 2: Real-Time Simulation Fidelity Tier")]
        public SimulationFidelityLevel FidelityLevel = SimulationFidelityLevel.Level2_6DoFBladeAndStrip;

        [Range(100, 1000)]
        [Tooltip("Internal RK4 integration frequency [Hz] for Fidelity Levels 1, 2, and 3.")]
        public int Rk4SubStepFrequencyHz = 400;

        [Header("Control Source (Default: External PX4 SITL via TCP 4560)")]
        [Tooltip("False by default: Unity + C# DLL acts purely as the 6-DOF & Aerodynamic Environment, while external PX4 SITL controls actuators via HIL_ACTUATOR_CONTROLS.")]
        public bool UseInternalFlightController = false;
        public bool AutoArmAndTakeoffOnStart = false;
        public float InitialTakeoffAltitudeMeters = 10.0f;

        [Header("Visual Rotor Transforms (Optional)")]
        public List<Transform> VisualRotorTransforms = new List<Transform>();

        [Header("Engineering Gizmos")]
        public bool DrawAerodynamicGizmos = true;
        public bool DrawSlipstreamWakeTubes = true;
        public bool DrawVlmWingPanels = true;

        private MultiFidelitySixDofEngine? _engine;
        private VirtualSensorSuite? _sensorSuite;
        private CascadedFlightController? _flightController;
        private Rigidbody? _unityRigidbody;
        private readonly double[] _controlBuffer16 = new double[16];

        public MultiFidelitySixDofEngine? Engine => _engine;
        public VirtualSensorSuite? SensorSuite => _sensorSuite;
        public CascadedFlightController? FlightController => _flightController;
        public VirtualSensorReadings LatestSensorReadings { get; private set; }

        public AircraftConfigSpec ActiveConfig =>
            ProfileAsset != null ? ProfileAsset.Config : InlineConfig;

        public BakedAeroDatabase ActiveBakedDatabase
        {
            get
            {
                if (ProfileAsset != null)
                {
                    if (ProfileAsset.BakedDatabase == null || ProfileAsset.BakedDatabase.RotorLumpedKt.Length == 0)
                        ProfileAsset.AnalyzeAndBake();
                    return ProfileAsset.BakedDatabase!;
                }
                if (InlineBakedDatabase == null || InlineBakedDatabase.RotorLumpedKt.Length == 0)
                    InlineBakedDatabase = StabilityAndTrimAnalyzer.AnalyzeAndBake(InlineConfig);
                return InlineBakedDatabase;
            }
        }

        private void Awake()
        {
            InitializeSimulationEngine();
        }

        private void Start()
        {
            if (AutoArmAndTakeoffOnStart && _flightController != null && _engine != null)
            {
                Vec3d euler = _engine.State.AttitudeBodyToNed.ToEulerZyx();
                _flightController.CommandTakeoff(_engine.State.PositionNed, euler.Z, InitialTakeoffAltitudeMeters);
            }
        }

        /// <summary>
        /// Stage 1 Helper: Extracts 3D bounding box from attached Unity MeshRenderers/SkinnedMeshRenderers
        /// and estimates inertia tensor, fuselage dimensions, and projected drag areas.
        /// </summary>
        [ContextMenu("Stage 1: Extract Geometry from Child Meshes & Bake Aero DB")]
        public void ExtractGeometryFromChildMeshesAndBake()
        {
            var renderers = GetComponentsInChildren<Renderer>();
            if (renderers.Length > 0)
            {
                Bounds combined = renderers[0].bounds;
                for (int i = 1; i < renderers.Length; i++)
                    combined.Encapsulate(renderers[i].bounds);

                // Unity size (X=Right/Y_FRD, Y=Up/-Z_FRD, Z=Forward/X_FRD)
                Vec3d sizeFrd = new Vec3d(combined.size.z, combined.size.x, combined.size.y);
                StabilityAndTrimAnalyzer.EstimateGeometryFromBoundingBox(ActiveConfig, sizeFrd, ActiveConfig.MassKg);
            }

            RunStage1AnalysisAndBake();
        }

        /// <summary>
        /// Executes Stage 1 Pre-Simulation Analysis and updates the baked database.
        /// </summary>
        [ContextMenu("Stage 1: Run Full Aerodynamic & Stability Analysis")]
        public BakedAeroDatabase RunStage1AnalysisAndBake()
        {
            BakedAeroDatabase db;
            if (ProfileAsset != null)
            {
                db = ProfileAsset.AnalyzeAndBake();
            }
            else
            {
                InlineBakedDatabase = StabilityAndTrimAnalyzer.AnalyzeAndBake(InlineConfig);
                db = InlineBakedDatabase;
            }

            if (_engine != null)
                _engine.Reconfigure(ActiveConfig, db);

            return db;
        }

        public void InitializeSimulationEngine()
        {
            var cfg = ActiveConfig;
            var db = ActiveBakedDatabase;

            _engine = new MultiFidelitySixDofEngine(cfg, db)
            {
                FidelityLevel = FidelityLevel
            };

            _sensorSuite = new VirtualSensorSuite(cfg.Sensors);
            _flightController = new CascadedFlightController();

            Vec3d initialPosNed = FrameConverter.UnityToNedPosition(new Vec3d(transform.position.x, transform.position.y, transform.position.z));
            Quatd initialRotNed = FrameConverter.UnityToNedRotation(new Quatd(transform.rotation.w, transform.rotation.x, transform.rotation.y, transform.rotation.z));
            _engine.ResetState(initialPosNed, initialRotNed);

            // Sample initial sensor state at t=0 so the very first HIL_SENSOR / HIL_GPS sent to PX4 has valid pressure, mag, and 3D GPS fix
            LatestSensorReadings = _sensorSuite.Sample(
                _engine.State, 0.004, _engine.OriginLatitudeDeg, _engine.OriginLongitudeDeg, _engine.OriginAltitudeMsl);

            ConfigureUnityRigidbodyForFidelity();
        }

        private void ConfigureUnityRigidbodyForFidelity()
        {
            _unityRigidbody = GetComponent<Rigidbody>();
            if (FidelityLevel == SimulationFidelityLevel.Level0_UnityRigidbody)
            {
                if (_unityRigidbody == null)
                    _unityRigidbody = gameObject.AddComponent<Rigidbody>();

                _unityRigidbody.mass = (float)ActiveConfig.MassKg;
                _unityRigidbody.useGravity = true;
                _unityRigidbody.isKinematic = false;
                _unityRigidbody.interpolation = RigidbodyInterpolation.Interpolate;
            }
            else if (_unityRigidbody != null)
            {
                _unityRigidbody.isKinematic = true;
            }
        }

        private void FixedUpdate()
        {
            if (_engine == null || _sensorSuite == null || _flightController == null)
                return;

            // If user changed FidelityLevel in Inspector at runtime, reconfigure Rigidbody mode
            if (_engine.FidelityLevel != FidelityLevel)
            {
                _engine.FidelityLevel = FidelityLevel;
                ConfigureUnityRigidbodyForFidelity();
            }

            // If managed by FlightSimulationManager in Lockstep mode, let manager step us
            if (_engine.ClockMode == SimulationClockMode.SitlLockstep)
                return;

            StepVehiclePhysicsAndControl(Time.fixedDeltaTime);
        }

        public void StepVehiclePhysicsAndControl(double dt)
        {
            if (_engine == null || _sensorSuite == null || _flightController == null)
                return;

            var cfg = ActiveConfig;
            var db = ActiveBakedDatabase;

            if (UseInternalFlightController)
            {
                _flightController.ComputeControlStep(dt, cfg, db, _engine.State, _controlBuffer16);
                _engine.SetActuatorControls(_controlBuffer16);
            }

            if (FidelityLevel == SimulationFidelityLevel.Level0_UnityRigidbody && _unityRigidbody != null)
            {
                Vector3 p = _unityRigidbody.position;
                Quaternion q = _unityRigidbody.rotation;
                Vector3 vLocal = transform.InverseTransformDirection(_unityRigidbody.linearVelocity);
                Vector3 wLocal = transform.InverseTransformDirection(_unityRigidbody.angularVelocity);

                _engine.EvaluateLevel0UnityRigidbodyWrench(
                    dt,
                    new Vec3d(p.x, p.y, p.z),
                    new Quatd(q.w, q.x, q.y, q.z),
                    new Vec3d(vLocal.x, vLocal.y, vLocal.z),
                    new Vec3d(wLocal.x, wLocal.y, wLocal.z),
                    out Vec3d unityForceLocal,
                    out Vec3d unityTorqueLocal);

                _unityRigidbody.AddRelativeForce(new Vector3((float)unityForceLocal.X, (float)unityForceLocal.Y, (float)unityForceLocal.Z), ForceMode.Force);
                _unityRigidbody.AddRelativeTorque(new Vector3((float)unityTorqueLocal.X, (float)unityTorqueLocal.Y, (float)unityTorqueLocal.Z), ForceMode.Force);
            }
            else
            {
                double maxSubStep = 1.0 / System.Math.Max(50, Rk4SubStepFrequencyHz);
                _engine.StepSimulation(dt, maxSubStep);

                // Sync Aerospace NED/FRD state to Unity Transform
                Vec3d uPos = FrameConverter.NedToUnityPosition(_engine.State.PositionNed);
                Quatd uRot = FrameConverter.NedToUnityRotation(_engine.State.AttitudeBodyToNed);

                transform.SetPositionAndRotation(
                    new Vector3((float)uPos.X, (float)uPos.Y, (float)uPos.Z),
                    new Quaternion((float)uRot.X, (float)uRot.Y, (float)uRot.Z, (float)uRot.W));
            }

            LatestSensorReadings = _sensorSuite.Sample(
                _engine.State, dt, _engine.OriginLatitudeDeg, _engine.OriginLongitudeDeg, _engine.OriginAltitudeMsl);

            UpdateVisualRotors((float)dt);
        }

        private void UpdateVisualRotors(float dt)
        {
            if (_engine == null || VisualRotorTransforms == null) return;
            int count = System.Math.Min(VisualRotorTransforms.Count, _engine.State.RotorRpm.Length);
            for (int i = 0; i < count; i++)
            {
                Transform? t = VisualRotorTransforms[i];
                if (t == null) continue;
                float degPerSec = (float)(_engine.State.RotorRpm[i] * 6.0 * ActiveConfig.Rotors[i].RotationDirection);
                t.Rotate(Vector3.up, degPerSec * dt, Space.Self);
            }
        }

        private void OnDrawGizmos()
        {
            if (!DrawAerodynamicGizmos) return;

            var cfg = ActiveConfig;
            var db = ActiveBakedDatabase;

            // 1. Draw Center of Gravity (Yellow sphere) & Neutral Point (Cyan sphere)
            Vec3d cgUnityLocal = FrameConverter.FrdToUnityVector(cfg.CenterOfGravityFrd);
            Vector3 cgWorld = transform.TransformPoint(new Vector3((float)cgUnityLocal.X, (float)cgUnityLocal.Y, (float)cgUnityLocal.Z));
            Gizmos.color = Color.yellow;
            Gizmos.DrawSphere(cgWorld, 0.03f);

            Vec3d npUnityLocal = FrameConverter.FrdToUnityVector(new Vec3d(db.NeutralPointXFrd, 0.0, 0.0));
            Vector3 npWorld = transform.TransformPoint(new Vector3((float)npUnityLocal.X, (float)npUnityLocal.Y, (float)npUnityLocal.Z));
            Gizmos.color = Color.cyan;
            Gizmos.DrawSphere(npWorld, 0.028f);
            Gizmos.DrawLine(cgWorld, npWorld);

            // 2. Draw Rotors, Thrust Vectors & Contracting Slipstream Wake Tubes
            for (int i = 0; i < cfg.Rotors.Count; i++)
            {
                var r = cfg.Rotors[i];
                Vec3d posU = FrameConverter.FrdToUnityVector(r.MountPositionFrd);
                Vec3d axisU = FrameConverter.FrdToUnityVector(r.ThrustAxisFrd).Normalized();

                Vector3 rWorld = transform.TransformPoint(new Vector3((float)posU.X, (float)posU.Y, (float)posU.Z));
                Vector3 tDirWorld = transform.TransformDirection(new Vector3((float)axisU.X, (float)axisU.Y, (float)axisU.Z));

                Gizmos.color = new Color(0.2f, 1.0f, 0.4f, 0.9f);
                Gizmos.DrawWireSphere(rWorld, (float)r.Radius);

                float thrustLen = (_engine != null && i < _engine.State.RotorThrustNewtons.Length)
                    ? (float)(_engine.State.RotorThrustNewtons[i] * 0.04)
                    : 0.25f;
                Gizmos.DrawLine(rWorld, rWorld + tDirWorld * thrustLen);

                if (DrawSlipstreamWakeTubes)
                {
                    Gizmos.color = new Color(0.3f, 0.7f, 1.0f, 0.35f);
                    Vector3 wakeEnd = rWorld - tDirWorld * (float)(r.Radius * 3.5);
                    Gizmos.DrawLine(rWorld, wakeEnd);
                    Gizmos.DrawWireSphere(wakeEnd, (float)(r.Radius * 0.78));
                }
            }

            // 3. Draw 3D VLM Wing Panels
            if (DrawVlmWingPanels && cfg.Wings != null && cfg.Wings.Count > 0)
            {
                var panels = VlmAndLiftingLineSolver.BuildPanels(cfg.Wings);
                Gizmos.color = new Color(1.0f, 0.55f, 0.1f, 0.8f);
                for (int i = 0; i < panels.Count; i++)
                {
                    var p = panels[i];
                    Vec3d aU = FrameConverter.FrdToUnityVector(p.VortexBoundLeftFrd);
                    Vec3d bU = FrameConverter.FrdToUnityVector(p.VortexBoundRightFrd);
                    Vec3d cU = FrameConverter.FrdToUnityVector(p.CollocationThreeQuarterFrd);

                    Vector3 wA = transform.TransformPoint(new Vector3((float)aU.X, (float)aU.Y, (float)aU.Z));
                    Vector3 wB = transform.TransformPoint(new Vector3((float)bU.X, (float)bU.Y, (float)bU.Z));
                    Vector3 wC = transform.TransformPoint(new Vector3((float)cU.X, (float)cU.Y, (float)cU.Z));

                    Gizmos.DrawLine(wA, wB);
                    Gizmos.DrawSphere(wC, 0.01f);
                }
            }
        }
    }
}
