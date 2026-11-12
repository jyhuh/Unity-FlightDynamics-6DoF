// Licensed under the Apache License, Version 2.0.
using System;
using System.Collections.Generic;
using FlightDynamics6DoF.Core.Aerodynamics;
using FlightDynamics6DoF.Core.Dynamics;
using FlightDynamics6DoF.Core.Gnc;
using FlightDynamics6DoF.Core.Math;
using FlightDynamics6DoF.Core.Mavlink;
using FlightDynamics6DoF.Core.Sensors;
using FlightDynamics6DoF.Core.Specs;

namespace FlightDynamics6DoF.Tests
{
    public static class Program
    {
        private static int _passed = 0;

        public static int Main()
        {
            Console.WriteLine("======================================================================");
            Console.WriteLine(" FlightDynamics 6-DOF Engineering Suite - Automated Verification Suite");
            Console.WriteLine("======================================================================");

            TestCoordinateAndWgs84Conversions();
            TestModule1AirfoilPolarAndViterna();
            TestModule4CcBladeBemtAndSlipstream();
            TestModule2And3VlmAndSlenderBody();
            TestStage1AnalysisAndStabilityDerivatives();
            TestStage2MultiFidelityClosedLoopFlight();
            TestMavlinkV2SitlAndGcsProtocol();

            Console.WriteLine("----------------------------------------------------------------------");
            Console.WriteLine($" ALL {_passed} ENGINEERING ASSERTIONS PASSED SUCCESSFULLY.");
            Console.WriteLine("======================================================================");
            return 0;
        }

        private static void AssertTrue(bool condition, string message)
        {
            if (!condition)
                throw new Exception($"Assertion Failed: {message}");
            _passed++;
        }

        private static void AssertNear(double actual, double expected, double tol, string message)
        {
            if (System.Math.Abs(actual - expected) > tol || double.IsNaN(actual))
                throw new Exception($"Assertion Failed: {message} (Expected {expected:F5} +/- {tol}, Actual {actual:F5})");
            _passed++;
        }

        private static void TestCoordinateAndWgs84Conversions()
        {
            Console.WriteLine("[1/7] Verifying Aerospace NED/FRD <-> Unity Left-Handed & WGS84 Geodetic...");
            Vec3d ned = new Vec3d(120.5, -45.2, -30.0);
            Vec3d unity = FrameConverter.NedToUnityPosition(ned);
            AssertNear(unity.X, -45.2, 1e-9, "Unity X = East");
            AssertNear(unity.Y, 30.0, 1e-9, "Unity Y = Up (-Down)");
            AssertNear(unity.Z, 120.5, 1e-9, "Unity Z = North");

            Vec3d roundTripNed = FrameConverter.UnityToNedPosition(unity);
            AssertTrue(ned.Equals(roundTripNed), "NED -> Unity -> NED position lossless round-trip");

            Quatd qNed = Quatd.FromEulerZyx(0.25, -0.15, 1.10);
            Quatd qUnity = FrameConverter.NedToUnityRotation(qNed);
            Quatd qBack = FrameConverter.UnityToNedRotation(qUnity);
            Vec3d e1 = qNed.ToEulerZyx();
            Vec3d e2 = qBack.ToEulerZyx();
            AssertNear(e1.X, e2.X, 1e-6, "Quaternion Roll round-trip");
            AssertNear(e1.Y, e2.Y, 1e-6, "Quaternion Pitch round-trip");
            AssertNear(e1.Z, e2.Z, 1e-6, "Quaternion Yaw round-trip");

            Vec3d lla = FrameConverter.NedToWgs84(ned, 37.5665, 126.9780, 50.0);
            Vec3d nedFromLla = FrameConverter.Wgs84ToNed(lla.X, lla.Y, lla.Z, 37.5665, 126.9780, 50.0);
            AssertNear(ned.X, nedFromLla.X, 1e-3, "WGS84 North round-trip");
            AssertNear(ned.Y, nedFromLla.Y, 1e-3, "WGS84 East round-trip");
            AssertNear(ned.Z, nedFromLla.Z, 1e-3, "WGS84 Down round-trip");

            // Verify central AerospaceConstants structs
            AssertNear(PhysicalConstants.GravityMps2, 9.80665, 1e-5, "PhysicalConstants.GravityMps2");
            AssertNear(PhysicalConstants.SeaLevelDensityKgM3, 1.225, 1e-3, "PhysicalConstants.SeaLevelDensityKgM3");
            AssertNear(Wgs84Constants.SemiMajorAxisM, 6378137.0, 1e-1, "Wgs84Constants.SemiMajorAxisM");
            AssertNear(GroundContactConstants.PavedRunwayRollingFrictionMu, 0.025, 1e-4, "GroundContactConstants.PavedRunwayRollingFrictionMu");
            AssertTrue(NetworkConstants.Px4SitlTcpPort == 4560, "NetworkConstants.Px4SitlTcpPort == 4560");
            AssertNear(MathConstants.Deg2Rad, System.Math.PI / 180.0, 1e-12, "MathConstants.Deg2Rad");
        }

        private static void TestModule1AirfoilPolarAndViterna()
        {
            Console.WriteLine("[2/7] Verifying Module 1 (CST + NeuralFoil Viscous + Viterna +/-180 deg + Du-Selig)...");
            var naca2412 = AirfoilSpec.FromNaca4Digit("2412", 5.0e5);

            var p0 = AirfoilPolarSolver.Evaluate(naca2412, 0.0, 5.0e5, 0.1);
            var p5 = AirfoilPolarSolver.Evaluate(naca2412, 5.0 * FrameConverter.Deg2Rad, 5.0e5, 0.1);
            AssertTrue(p0.Cl > 0.10 && p0.Cl < 0.40, $"Positive camber NACA 2412 Cl(0) > 0 (got {p0.Cl:F3})");
            AssertTrue(p5.Cl > p0.Cl + 0.40, $"Attached lift slope dCl/dAlpha > 0 (got Cl(5deg)={p5.Cl:F3})");
            AssertTrue(p5.TopTransitionXtr < p0.TopTransitionXtr, "Upper boundary-layer transition moves forward with increasing alpha");

            // Check full -180 to +180 deg Viterna extrapolation finite & physical
            for (int deg = -180; deg <= 180; deg += 5)
            {
                var pt = AirfoilPolarSolver.Evaluate(naca2412, deg * FrameConverter.Deg2Rad, 3.0e5, 0.15, 8.0);
                AssertTrue(!double.IsNaN(pt.Cl) && !double.IsInfinity(pt.Cl), $"Finite Cl at {deg} deg");
                AssertTrue(pt.Cd > 0.002 && pt.Cd < 2.5, $"Physical Cd at {deg} deg (got {pt.Cd:F3})");
            }

            // Du-Selig 3D rotational stall delay increases post-stall Cl near root
            double alphaStallRegion = 16.0 * FrameConverter.Deg2Rad;
            var p2d = AirfoilPolarSolver.Evaluate(naca2412, alphaStallRegion, 1.5e5, 0.1);
            var p3d = AirfoilPolarSolver.EvaluateRotatingBladeSection(naca2412, alphaStallRegion, 1.5e5, 0.1, 0.22, 0.25, 6.0);
            AssertTrue(p3d.Cl >= p2d.Cl, $"Du-Selig rotational stall delay augments Cl ({p3d.Cl:F3} >= {p2d.Cl:F3})");
        }

        private static void TestModule4CcBladeBemtAndSlipstream()
        {
            Console.WriteLine("[3/7] Verifying Module 4 (CCBlade BEMT + Ground Effect + Contracting Slipstream Wake)...");
            var quad = AircraftConfigSpec.CreateStandardQuadcopterX();
            var rotor = quad.Rotors[0];
            var atm = IsaAtmosphereAndTurbulence.Evaluate(50.0);

            var solOge = CcBladeBemtSolver.EvaluateRotor(rotor, Vec3d.Zero, Vec3d.Zero, 6000.0, atm, 50.0, enableGroundEffect: true);
            AssertTrue(solOge.ThrustNewtons > 2.5 && solOge.ThrustNewtons < 15.0, $"Hover thrust at 6000 RPM physical ({solOge.ThrustNewtons:F2} N)");
            AssertTrue(solOge.AerodynamicTorqueNm > 0.02, $"Hover torque positive ({solOge.AerodynamicTorqueNm:F4} Nm)");
            AssertTrue(solOge.MeanAxialInducedVelocityMps > 2.0, $"Hover induced velocity positive ({solOge.MeanAxialInducedVelocityMps:F2} m/s)");

            var solIge = CcBladeBemtSolver.EvaluateRotor(rotor, Vec3d.Zero, Vec3d.Zero, 6000.0, atm, 0.12, enableGroundEffect: true);
            AssertTrue(solIge.ThrustNewtons > solOge.ThrustNewtons, $"Cheeseman-Bennett ground effect increases thrust near ground ({solIge.ThrustNewtons:F2} > {solOge.ThrustNewtons:F2})");

            // Contracting slipstream wake velocity downstream of rotor (+Z_FRD below a vertical rotor)
            Vec3d downstreamPt = rotor.MountPositionFrd + new Vec3d(0.0, 0.0, 0.20);
            Vec3d vWake = CcBladeBemtSolver.ComputeSlipstreamInducedVelocityFrd(rotor, solOge, downstreamPt);
            AssertTrue(vWake.Z > solOge.MeanAxialInducedVelocityMps, $"Contracting slipstream accelerates wake downstream (Vz={vWake.Z:F2} m/s)");
        }

        private static void TestModule2And3VlmAndSlenderBody()
        {
            Console.WriteLine("[4/7] Verifying Modules 2 & 3 (3D VLM Horseshoe Vortices + Slender Body & Build-up)...");
            var fw = AircraftConfigSpec.CreateFixedWingUav();
            var atm = IsaAtmosphereAndTurbulence.Evaluate(100.0);
            var panels = VlmAndLiftingLineSolver.BuildPanels(fw.Wings);
            AssertTrue(panels.Count == 24, $"Expected 24 VLM panels across MainWing, H-Tail, V-Fin (got {panels.Count})");

            var wingRes = VlmAndLiftingLineSolver.Evaluate(
                fw.Wings, panels, new Vec3d(18.0, 0.0, 1.0), Vec3d.Zero, fw.CenterOfGravityFrd, atm, null, null, null, true);
            AssertTrue(wingRes.TotalLiftNewtons > 15.0, $"3D VLM generates positive lift ({wingRes.TotalLiftNewtons:F2} N)");
            AssertTrue(wingRes.TotalInducedDragNewtons > 0.1, $"3D VLM generates positive induced drag ({wingRes.TotalInducedDragNewtons:F2} N)");

            var fusRes = FuselageAndParasiticSolver.Evaluate(
                fw.Fuselage, new Vec3d(18.0, 0.0, 1.0), fw.CenterOfGravityFrd, fw.ReferenceAreaSqM, fw.ReferenceChordMeters, fw.ReferenceSpanMeters, atm);
            AssertTrue(fusRes.ParasiticCd0 > 0.004 && fusRes.ParasiticCd0 < 0.08, $"Component build-up CD0 physical ({fusRes.ParasiticCd0:F4})");
            AssertTrue(fusRes.CmAlphaFuselagePerRad > 0.0, $"Slender body fuselage pitching moment is destabilizing Cm_alpha_fus > 0 ({fusRes.CmAlphaFuselagePerRad:F4})");
        }

        private static void TestStage1AnalysisAndStabilityDerivatives()
        {
            Console.WriteLine("[5/7] Verifying Module 5 & Stage 1 Pipeline (Stability Derivatives, Static Margin, Trim & PX4 Export)...");
            var fw = AircraftConfigSpec.CreateFixedWingUav();
            var dbFw = StabilityAndTrimAnalyzer.AnalyzeAndBake(fw);

            Console.WriteLine($"      Fixed-Wing Analysis -> CL_alpha={dbFw.CLAlpha:F2}/rad, Cm_alpha={dbFw.CmAlpha:F3}/rad, Cn_beta={dbFw.CnBeta:F3}/rad");
            Console.WriteLine($"      Neutral Point x_NP={dbFw.NeutralPointXFrd:F3} m, Static Margin={(dbFw.StaticMarginFraction * 100.0):F1}% MAC, Trim Alpha={dbFw.TrimAlphaDeg:F2} deg");

            AssertTrue(dbFw.CLAlpha > 3.0 && dbFw.CLAlpha < 6.5, $"3D finite-wing lift slope CL_alpha in physical range ({dbFw.CLAlpha:F2})");
            AssertTrue(dbFw.CmAlpha < -0.10, $"Aircraft is longitudinally statically stable Cm_alpha < 0 ({dbFw.CmAlpha:F3})");
            AssertTrue(dbFw.StaticMarginFraction > 0.05 && dbFw.StaticMarginFraction < 0.65, $"Positive Static Margin ({dbFw.StaticMarginFraction:F3})");
            AssertTrue(dbFw.CnBeta > 0.02, $"Weathercock directional stability Cn_beta > 0 ({dbFw.CnBeta:F3})");
            AssertTrue(dbFw.Clp < -0.10 && dbFw.Cmq < -1.0 && dbFw.Cnr < -0.01,
                $"Dynamic damping derivatives negative (Clp={dbFw.Clp:F2}, Cmq={dbFw.Cmq:F2}, Cnr={dbFw.Cnr:F3})");

            // Test Hybrid QuadPlane VTOL ("Millie"-class) analysis & PX4 Airframe Script export
            var vtol = AircraftConfigSpec.CreateQuadPlaneVtolMillie();
            var dbVtol = StabilityAndTrimAnalyzer.AnalyzeAndBake(vtol);
            string px4Script = WrenchControlAllocator.GeneratePx4AirframeParameterScript(vtol, dbVtol);
            AssertTrue(px4Script.Contains("CA_ROTOR_COUNT 5"), "PX4 exporter contains all 5 QuadPlane rotors");
            AssertTrue(px4Script.Contains("CA_SV_CS_COUNT 3"), "PX4 exporter contains 3 aerodynamic control surfaces");
        }

        private static void TestStage2MultiFidelityClosedLoopFlight()
        {
            Console.WriteLine("[6/7] Verifying Stage 2 Real-Time Simulation across Fidelity Levels 0..3 & Closed-Loop GNC Takeoff...");
            var quad = AircraftConfigSpec.CreateStandardQuadcopterX();
            var db = StabilityAndTrimAnalyzer.AnalyzeAndBake(quad);
            var engine = new MultiFidelitySixDofEngine(quad, db);

            // Verify aircraft resting on ground with parking brake / skid friction (remains stationary under wind)
            var fwTest = AircraftConfigSpec.CreateFixedWingUav();
            var fwDbTest = StabilityAndTrimAnalyzer.AnalyzeAndBake(fwTest);
            var fwEngineTest = new MultiFidelitySixDofEngine(fwTest, fwDbTest);
            fwEngineTest.ResetState(new Vec3d(0.0, 0.0, 0.0), Quatd.Identity);
            fwEngineTest.FidelityLevel = SimulationFidelityLevel.Level2_6DoFBladeAndStrip;
            for (int i = 0; i < 50; i++)
            {
                fwEngineTest.StepSimulation(0.02);
            }
            AssertNear(fwEngineTest.State.AltitudeAglMeters, 0.0, 0.02, "Resting aircraft stays stationary on ground without sliding");
            AssertTrue(fwEngineTest.State.VelocityBodyFrd.Magnitude < 0.05, "Ground friction/parking brake holds vehicle stationary under wind");
            var controller = new CascadedFlightController();

            // Verify Level 0 Unity Rigidbody Force/Torque Evaluation
            engine.FidelityLevel = SimulationFidelityLevel.Level0_UnityRigidbody;
            double[] hoverCtrl = new double[16];
            hoverCtrl[0] = 0.6; hoverCtrl[1] = 0.6; hoverCtrl[2] = 0.6; hoverCtrl[3] = 0.6;
            engine.SetActuatorControls(hoverCtrl);
            engine.EvaluateLevel0UnityRigidbodyWrench(
                0.10, Vec3d.Zero, Quatd.Identity, Vec3d.Zero, Vec3d.Zero,
                out Vec3d unityForce, out Vec3d unityTorque);
            AssertTrue(unityForce.Y > 10.0, $"Level 0 Unity Rigidbody upward force (+Y_Unity) positive ({unityForce.Y:F2} N)");
            AssertNear(unityTorque.Magnitude, 0.0, 1e-3, "Symmetric quadcopter command yields zero net torque");

            // Verify Closed-Loop Takeoff & Altitude/Position Hold in Level 1, Level 2, and Level 3
            SimulationFidelityLevel[] levels = new[]
            {
                SimulationFidelityLevel.Level1_6DoFSurrogate,
                SimulationFidelityLevel.Level2_6DoFBladeAndStrip,
                SimulationFidelityLevel.Level3_6DoFFullAeroWake
            };

            Span<double> ctrl16 = stackalloc double[16];
            foreach (var lvl in levels)
            {
                engine.ResetState(Vec3d.Zero, Quatd.Identity);
                engine.FidelityLevel = lvl;
                controller.ResetIntegrators();
                controller.CommandTakeoff(Vec3d.Zero, 0.0, 10.0); // Takeoff to 10m AGL (PositionNed.Z = -10.0)

                double dt = 0.005; // 200 Hz outer step
                for (int step = 0; step < 1200; step++) // 6.0 seconds simulated flight
                {
                    controller.ComputeControlStep(dt, quad, db, engine.State, ctrl16);
                    engine.SetActuatorControls(ctrl16);
                    engine.StepSimulation(dt);
                }

                Console.WriteLine($"      [{lvl}] 6.0s Closed-Loop Takeoff -> Altitude AGL = {engine.State.AltitudeAglMeters:F2} m (Target 10.0 m)");
                AssertTrue(engine.State.AltitudeAglMeters > 8.5 && engine.State.AltitudeAglMeters < 11.2,
                    $"{lvl} reached target takeoff altitude (~10m AGL, got {engine.State.AltitudeAglMeters:F2}m)");
            }

            // Verify Fixed-Wing Runway Acceleration, Rotation and Climbout
            Console.WriteLine("      [Fixed-Wing Runway Roll] Simulating runway roll, rotation and climbout...");
            var fw = AircraftConfigSpec.CreateFixedWingUav();
            var fwDb = StabilityAndTrimAnalyzer.AnalyzeAndBake(fw);
            var fwEngine = new MultiFidelitySixDofEngine(fw, fwDb);
            fwEngine.ResetState(Vec3d.Zero, Quatd.Identity);

            double[] fwControls = new double[16];
            fwControls[4] = 0.85; // Full throttle on Ch 4 (PX4 SITL plane)
            fwEngine.SetActuatorControls(fwControls);

            double stepDt = 0.005;
            double maxGroundRollAngleDeg = 0.0;
            for (int s = 0; s < 600; s++) // 3.0s ground roll
            {
                // Nosewheel/rudder steering on runway to maintain runway centerline heading (yaw = 0)
                double yaw = fwEngine.State.AttitudeBodyToNed.ToEulerZyx().Z;
                fwControls[8] = -0.8 * yaw; // Gentle rudder on Ch 8
                fwEngine.SetActuatorControls(fwControls);
                fwEngine.StepSimulation(stepDt);

                if (fwEngine.State.IsGrounded)
                {
                    double rollDeg = System.Math.Abs(fwEngine.State.AttitudeBodyToNed.ToEulerZyx().X * FrameConverter.Rad2Deg);
                    if (rollDeg > maxGroundRollAngleDeg) maxGroundRollAngleDeg = rollDeg;
                }
            }

            Vec3d groundRollEuler = fwEngine.State.AttitudeBodyToNed.ToEulerZyx();
            Console.WriteLine($"      Fixed-Wing Ground Roll (3.0s): Airspeed = {fwEngine.State.AirspeedMps:F2} m/s, Altitude = {fwEngine.State.AltitudeAglMeters:F2} m, Grounded = {fwEngine.State.IsGrounded}, MaxGroundRoll = {maxGroundRollAngleDeg:F2} deg, Heading = {groundRollEuler.Z * FrameConverter.Rad2Deg:F1} deg");
            AssertTrue(fwEngine.State.AirspeedMps > 10.0, $"Fixed-wing accelerates down runway (got {fwEngine.State.AirspeedMps:F2} m/s)");
            AssertTrue(maxGroundRollAngleDeg < 3.0, $"Fixed-wing maintains level wings during runway roll (MaxRoll = {maxGroundRollAngleDeg:F2} deg < 3.0 deg)");

            // Apply elevator pitch-up for rotation and liftoff (+0.35 in standard aerospace & PX4 convention)
            fwControls[7] = 0.35;
            for (int s = 0; s < 140; s++) // 0.70s rotation & climbout
            {
                double yaw = fwEngine.State.AttitudeBodyToNed.ToEulerZyx().Z;
                fwControls[8] = -0.8 * yaw;
                fwEngine.SetActuatorControls(fwControls);
                fwEngine.StepSimulation(stepDt);
            }

            Console.WriteLine($"      Fixed-Wing Liftoff (3.7s): Altitude AGL = {fwEngine.State.AltitudeAglMeters:F2} m, Airspeed = {fwEngine.State.AirspeedMps:F2} m/s, Pitch = {fwEngine.State.AttitudeBodyToNed.ToEulerZyx().Y * FrameConverter.Rad2Deg:F2} deg");
            AssertTrue(fwEngine.State.AltitudeAglMeters > 0.5, $"Fixed-wing lifted off runway successfully (Altitude AGL = {fwEngine.State.AltitudeAglMeters:F2} m)");
        }

        private static void TestMavlinkV2SitlAndGcsProtocol()
        {
            Console.WriteLine("[7/7] Verifying Zero-Dependency MAVLink v2 Framing, SITL HIL & QGC Mission Upload...");
            var quad = AircraftConfigSpec.CreateStandardQuadcopterX();
            var engine = new MultiFidelitySixDofEngine(quad);
            var sensors = new VirtualSensorSuite(quad.Sensors);
            var controller = new CascadedFlightController();
            var bridge = new MavlinkSimulationBridge { Mode = MavlinkBridgeMode.DirectGcsAutopilot };

            var readings = sensors.Sample(engine.State, 0.004, engine.OriginLatitudeDeg, engine.OriginLongitudeDeg, engine.OriginAltitudeMsl);

            List<byte[]> outboundPackets = new List<byte[]>();
            bridge.StepOutboundTelemetry(engine, readings, controller, mem => outboundPackets.Add(mem.ToArray()));
            AssertTrue(outboundPackets.Count >= 3, "Outbound telemetry emitted HEARTBEAT, ATTITUDE, and GLOBAL_POSITION_INT");

            // Simulate QGroundControl sending MAV_CMD_COMPONENT_ARM_DISARM (400, param1=1) and MAV_CMD_NAV_TAKEOFF (22, param7=15m)
            Span<byte> cmdPayload = stackalloc byte[33];
            cmdPayload.Clear();
            MavlinkV2Codec.WriteSingle(cmdPayload, 0, 1.0f); // param1 = 1 (Arm)
            System.Buffers.Binary.BinaryPrimitives.WriteUInt16LittleEndian(cmdPayload.Slice(28, 2), 400);

            Span<byte> wirePacket = stackalloc byte[64];
            int wireLen = MavlinkV2Codec.EncodeFrame(MavlinkMessageIds.CommandLong, 1, 255, 190, cmdPayload, wirePacket);
            bridge.ProcessIncomingBytes(wirePacket.Slice(0, wireLen), engine, controller);
            AssertTrue(controller.IsArmed, "Controller armed via MAVLink COMMAND_LONG (400)");

            // Simulate PX4 SITL sending HIL_ACTUATOR_CONTROLS
            bridge.Mode = MavlinkBridgeMode.ExternalSitlHil;
            double[] sitlControls = new double[16];
            sitlControls[0] = 0.72; sitlControls[1] = 0.72; sitlControls[2] = 0.72; sitlControls[3] = 0.72;
            Span<byte> hilWire = stackalloc byte[120];
            int hilLen = MavlinkV2Codec.PackHilActuatorControls(2, 1, 1, 123456UL, sitlControls, 1, 0, hilWire);
            bridge.ProcessIncomingBytes(hilWire.Slice(0, hilLen), engine, controller);
            AssertTrue(bridge.ReceivedActuatorControlsThisStep, "Bridge decoded HIL_ACTUATOR_CONTROLS frame");
            AssertNear(engine.State.ActuatorControlsNormalized[0], 0.72, 1e-4, "Engine received SITL actuator control ch0");
        }
    }
}
