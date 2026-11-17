// Licensed under the Apache License, Version 2.0.
using System;
using System.Buffers.Binary;
using System.Collections.Generic;
using FlightDynamics6DoF.Core.Dynamics;
using FlightDynamics6DoF.Core.Gnc;
using FlightDynamics6DoF.Core.Math;
using FlightDynamics6DoF.Core.Sensors;

namespace FlightDynamics6DoF.Core.Mavlink
{
    public enum MavlinkBridgeMode
    {
        /// <summary>
        /// Connects to external PX4 or ArduPilot SITL (sends HIL_SENSOR / HIL_GPS, receives HIL_ACTUATOR_CONTROLS).
        /// </summary>
        ExternalSitlHil = 0,

        /// <summary>
        /// Standalone Autopilot mode: connects directly to QGroundControl or Mission Planner over UDP (14550),
        /// streaming telemetry and accepting Arm/Disarm, Mode, Parameter, and Mission Waypoint uploads into the built-in GNC.
        /// </summary>
        DirectGcsAutopilot = 1
    }

    /// <summary>
    /// Transport-agnostic MAVLink v2 Protocol Engine bridging <see cref="MultiFidelitySixDofEngine"/>,
    /// <see cref="VirtualSensorSuite"/>, and <see cref="CascadedFlightController"/> with PX4/ArduPilot SITL or QGroundControl.
    /// </summary>
    public class MavlinkSimulationBridge
    {
        private readonly MavlinkV2StreamParser _parser = new MavlinkV2StreamParser();
        private readonly List<MissionWaypointNed> _uploadingMission = new List<MissionWaypointNed>();
        private ushort _expectedMissionCount = 0;
        private byte _txSequence = 0;
        private double _lastHeartbeatTimeSec = -1.0;
        private double _lastTelemetryTimeSec = -1.0;
        private readonly byte[] _packetBuf = new byte[280];
        private readonly byte[] _replyBuf = new byte[280];

        public MavlinkBridgeMode Mode { get; set; } = MavlinkBridgeMode.ExternalSitlHil;
        public byte SystemId { get; set; } = 1;
        public byte ComponentId { get; set; } = 1;
        public bool ReceivedActuatorControlsThisStep { get; private set; }
        public bool IsExternalSitlArmed { get; private set; }
        public ulong LastReceivedHilTimestampUsec { get; private set; }
        public long TotalFramesSent { get; private set; }
        public long TotalFramesReceived { get; private set; }
        public long TotalHilActuatorFramesReceived { get; private set; }

        /// <summary>
        /// Processes incoming raw network bytes (from UDP or TCP socket) and updates either the SITL actuator controls
        /// or the built-in <see cref="CascadedFlightController"/> state, writing any immediate reply packets to <paramref name="sendCallback"/>.
        /// </summary>
        public void ProcessIncomingBytes(
            ReadOnlySpan<byte> data,
            MultiFidelitySixDofEngine engine,
            CascadedFlightController controller,
            Action<ReadOnlyMemory<byte>>? sendCallback = null)
        {
            ReceivedActuatorControlsThisStep = false;

            for (int i = 0; i < data.Length; i++)
            {
                if (_parser.PushByte(data[i], out MavlinkV2Frame frame))
                {
                    TotalFramesReceived++;
                    HandleReceivedFrame(frame, engine, controller, _replyBuf, sendCallback);
                }
            }
        }

        /// <summary>
        /// Generates outbound MAVLink v2 packets for the current simulation step and invokes <paramref name="sendCallback"/>.
        /// </summary>
        public void StepOutboundTelemetry(
            MultiFidelitySixDofEngine engine,
            VirtualSensorReadings sensorReadings,
            CascadedFlightController controller,
            Action<ReadOnlyMemory<byte>> sendCallback)
        {
            if (sendCallback == null) return;
            double now = engine.State.SimTimeSec;

            // 1. 1 Hz Heartbeat
            if (_lastHeartbeatTimeSec < 0.0 || (now - _lastHeartbeatTimeSec) >= 1.0)
            {
                _lastHeartbeatTimeSec = now;
                byte baseMode = (byte)(0x80 | (controller.IsArmed ? 0x80 : 0x00) | 0x01);
                byte sysStatus = (byte)(controller.IsArmed ? 4 : 3); // 4: MAV_STATE_ACTIVE, 3: STANDBY
                int len = MavlinkV2Codec.PackHeartbeat(_txSequence++, SystemId, ComponentId, 2, 12, baseMode, (uint)controller.CurrentMode, sysStatus, _packetBuf);
                sendCallback(new ReadOnlyMemory<byte>(_packetBuf, 0, len));
                TotalFramesSent++;
            }

            if (Mode == MavlinkBridgeMode.ExternalSitlHil)
            {
                // High-rate HIL_SENSOR (#107) every step to drive PX4 simulator_mavlink clock & EKF2
                int lenSensor = MavlinkV2Codec.PackHilSensor(_txSequence++, SystemId, ComponentId, sensorReadings, _packetBuf);
                sendCallback(new ReadOnlyMemory<byte>(_packetBuf, 0, lenSensor));
                TotalFramesSent++;

                // Ground-truth HIL_STATE_QUATERNION (#115) for PX4 SITL logging & estimator comparison
                Vec3d lla = FrameConverter.NedToWgs84(
                    engine.State.PositionNed, engine.OriginLatitudeDeg, engine.OriginLongitudeDeg, engine.OriginAltitudeMsl);
                int lenState = MavlinkV2Codec.PackHilStateQuaternion(
                    _txSequence++, SystemId, ComponentId, sensorReadings.TimestampMicroseconds,
                    engine.State, lla.X, lla.Y, lla.Z, _packetBuf);
                sendCallback(new ReadOnlyMemory<byte>(_packetBuf, 0, lenState));
                TotalFramesSent++;

                if (sensorReadings.GpsUpdatedThisStep)
                {
                    int lenGps = MavlinkV2Codec.PackHilGps(_txSequence++, SystemId, ComponentId, sensorReadings, _packetBuf);
                    sendCallback(new ReadOnlyMemory<byte>(_packetBuf, 0, lenGps));
                    TotalFramesSent++;
                }
            }
            else
            {
                // Direct QGroundControl / Mission Planner Telemetry at 20 Hz
                if (_lastTelemetryTimeSec < 0.0 || (now - _lastTelemetryTimeSec) >= 0.05)
                {
                    _lastTelemetryTimeSec = now;
                    uint bootMs = (uint)(now * 1000.0);

                    int lenAtt = MavlinkV2Codec.PackAttitude(
                        _txSequence++, SystemId, ComponentId, bootMs,
                        engine.State.AttitudeBodyToNed, engine.State.AngularVelocityBodyFrd, _packetBuf);
                    sendCallback(new ReadOnlyMemory<byte>(_packetBuf, 0, lenAtt));
                    TotalFramesSent++;

                    Vec3d lla = FrameConverter.NedToWgs84(
                        engine.State.PositionNed, engine.OriginLatitudeDeg, engine.OriginLongitudeDeg, engine.OriginAltitudeMsl);
                    Vec3d euler = engine.State.AttitudeBodyToNed.ToEulerZyx();

                    int lenGpos = MavlinkV2Codec.PackGlobalPositionInt(
                        _txSequence++, SystemId, ComponentId, bootMs,
                        lla.X, lla.Y, lla.Z, -engine.State.PositionNed.Z,
                        engine.State.VelocityNed, euler.Z * FrameConverter.Rad2Deg, _packetBuf);
                    sendCallback(new ReadOnlyMemory<byte>(_packetBuf, 0, lenGpos));
                    TotalFramesSent++;
                }
            }
        }

        private void HandleReceivedFrame(
            MavlinkV2Frame frame,
            MultiFidelitySixDofEngine engine,
            CascadedFlightController controller,
            byte[] replyBuf,
            Action<ReadOnlyMemory<byte>>? sendCallback)
        {
            switch (frame.MessageId)
            {
                case MavlinkMessageIds.HilActuatorControls:
                {
                    Span<double> controls = stackalloc double[16];
                    MavlinkV2Codec.UnpackHilActuatorControls(frame.Payload, controls, out ulong timeUsec, out byte modeByte);
                    IsExternalSitlArmed = (modeByte & 0x80) != 0 || controls[0] > 0.01 || controls[3] > 0.01 || controls[4] > 0.01;
                    engine.SetActuatorControls(controls);
                    LastReceivedHilTimestampUsec = timeUsec;
                    ReceivedActuatorControlsThisStep = true;
                    TotalHilActuatorFramesReceived++;
                    break;
                }

                case MavlinkMessageIds.CommandLong:
                {
                    MavlinkV2Codec.UnpackCommandLong(
                        frame.Payload, out ushort cmd, out float p1, out float p2, out float p5, out float p6, out float p7);

                    Vec3d euler = engine.State.AttitudeBodyToNed.ToEulerZyx();
                    if (cmd == 400) // MAV_CMD_COMPONENT_ARM_DISARM
                    {
                        if (p1 >= 0.5f)
                            controller.Arm(engine.State.PositionNed, euler.Z);
                        else
                            controller.Disarm();
                    }
                    else if (cmd == 22) // MAV_CMD_NAV_TAKEOFF
                    {
                        double alt = p7 > 0.5f ? p7 : 10.0;
                        controller.CommandTakeoff(engine.State.PositionNed, euler.Z, alt);
                    }
                    else if (cmd == 20) // MAV_CMD_NAV_RETURN_TO_LAUNCH
                    {
                        controller.CommandReturnToLaunch();
                    }
                    else if (cmd == 21) // MAV_CMD_NAV_LAND
                    {
                        controller.CommandAutoLand(engine.State.PositionNed);
                    }
                    else if (cmd == 176) // MAV_CMD_DO_SET_MODE
                    {
                        int customMode = (int)p2;
                        if (Enum.IsDefined(typeof(AutopilotFlightMode), customMode))
                            controller.CurrentMode = (AutopilotFlightMode)customMode;
                    }

                    if (sendCallback != null)
                    {
                        int ackLen = MavlinkV2Codec.PackCommandAck(_txSequence++, SystemId, ComponentId, cmd, 0, replyBuf);
                        sendCallback(new ReadOnlyMemory<byte>(replyBuf, 0, ackLen));
                        TotalFramesSent++;
                    }
                    break;
                }

                case MavlinkMessageIds.ParamRequestList:
                {
                    if (sendCallback != null)
                    {
                        SendAllControllerParameters(controller, replyBuf, sendCallback);
                    }
                    break;
                }

                case MavlinkMessageIds.ParamSet:
                {
                    MavlinkV2Codec.UnpackParamSet(frame.Payload, out string paramId, out float val);
                    ApplyParameterUpdate(controller, paramId, val);
                    if (sendCallback != null)
                    {
                        int pLen = MavlinkV2Codec.PackParamValue(_txSequence++, SystemId, ComponentId, paramId, val, 6, 0, replyBuf);
                        sendCallback(new ReadOnlyMemory<byte>(replyBuf, 0, pLen));
                        TotalFramesSent++;
                    }
                    break;
                }

                case MavlinkMessageIds.MissionCount:
                {
                    Span<byte> full = stackalloc byte[4];
                    full.Clear();
                    frame.Payload.AsSpan(0, System.Math.Min(frame.PayloadLength, 4)).CopyTo(full);
                    _expectedMissionCount = BinaryPrimitives.ReadUInt16LittleEndian(full.Slice(0, 2));
                    _uploadingMission.Clear();

                    if (_expectedMissionCount > 0 && sendCallback != null)
                    {
                        int reqLen = PackMissionRequestInt(_txSequence++, SystemId, ComponentId, 0, replyBuf);
                        sendCallback(new ReadOnlyMemory<byte>(replyBuf, 0, reqLen));
                        TotalFramesSent++;
                    }
                    break;
                }

                case MavlinkMessageIds.MissionItemInt:
                {
                    Span<byte> full = stackalloc byte[38];
                    full.Clear();
                    frame.Payload.AsSpan(0, System.Math.Min(frame.PayloadLength, 38)).CopyTo(full);

                    float holdTime = MavlinkV2Codec.ReadSingle(full, 0);
                    float acceptRadius = MavlinkV2Codec.ReadSingle(full, 4);
                    float yawDeg = MavlinkV2Codec.ReadSingle(full, 12);
                    int latInt = BinaryPrimitives.ReadInt32LittleEndian(full.Slice(16, 4));
                    int lonInt = BinaryPrimitives.ReadInt32LittleEndian(full.Slice(20, 4));
                    float altMeters = MavlinkV2Codec.ReadSingle(full, 24);
                    ushort seqIdx = BinaryPrimitives.ReadUInt16LittleEndian(full.Slice(28, 2));

                    Vec3d wpNed = FrameConverter.Wgs84ToNed(
                        latInt / 1e7, lonInt / 1e7, engine.OriginAltitudeMsl + altMeters,
                        engine.OriginLatitudeDeg, engine.OriginLongitudeDeg, engine.OriginAltitudeMsl);

                    _uploadingMission.Add(new MissionWaypointNed
                    {
                        PositionNed = wpNed,
                        YawRad = float.IsNaN(yawDeg) ? 0.0 : yawDeg * FrameConverter.Deg2Rad,
                        AcceptanceRadiusMeters = acceptRadius > 0.1f ? acceptRadius : 2.0,
                        HoldTimeSec = holdTime
                    });

                    if (_uploadingMission.Count < _expectedMissionCount && sendCallback != null)
                    {
                        int reqLen = PackMissionRequestInt(_txSequence++, SystemId, ComponentId, (ushort)(seqIdx + 1), replyBuf);
                        sendCallback(new ReadOnlyMemory<byte>(replyBuf, 0, reqLen));
                        TotalFramesSent++;
                    }
                    else
                    {
                        controller.SetMissionWaypoints(_uploadingMission);
                        controller.CurrentMode = AutopilotFlightMode.WaypointMission;
                        if (sendCallback != null)
                        {
                            Span<byte> ackP = stackalloc byte[3];
                            ackP.Clear();
                            int ackLen = MavlinkV2Codec.EncodeFrame(MavlinkMessageIds.MissionAck, _txSequence++, SystemId, ComponentId, ackP, replyBuf);
                            sendCallback(new ReadOnlyMemory<byte>(replyBuf, 0, ackLen));
                            TotalFramesSent++;
                        }
                    }
                    break;
                }
            }
        }

        private void SendAllControllerParameters(
            CascadedFlightController c,
            byte[] buf,
            Action<ReadOnlyMemory<byte>> sendCallback)
        {
            var list = c.ParameterTable.GetMavlinkParameterList();

            for (ushort i = 0; i < list.Count; i++)
            {
                int len = MavlinkV2Codec.PackParamValue(_txSequence++, SystemId, ComponentId, list[i].Name, list[i].Value, (ushort)list.Count, i, buf);
                sendCallback(new ReadOnlyMemory<byte>(buf, 0, len));
                TotalFramesSent++;
            }
        }

        private static void ApplyParameterUpdate(CascadedFlightController c, string name, float val)
        {
            if (c.ParameterTable.TrySetParameter(name, val))
            {
                c.ApplyParameterTable(c.ParameterTable);
                return;
            }

            switch (name)
            {
                case "MPC_XY_P": c.KpPosXY = val; break;
                case "MPC_Z_P": c.KpPosZ = val; break;
                case "MPC_XY_VEL_P": c.KpVelXY = val; break;
                case "MPC_Z_VEL_P": c.KpVelZ = val; break;
                case "MC_ROLL_P": c.KpAttitudeRollPitch = val; break;
                case "MC_ROLLRATE_P": c.KpRateRollPitch = val; break;
            }
        }

        private static int PackMissionRequestInt(byte seq, byte sysId, byte compId, ushort missionSeq, Span<byte> outPacket)
        {
            Span<byte> p = stackalloc byte[4];
            BinaryPrimitives.WriteUInt16LittleEndian(p.Slice(0, 2), missionSeq);
            p[2] = 255; // target_system (GCS)
            p[3] = 190; // target_component
            return MavlinkV2Codec.EncodeFrame(MavlinkMessageIds.MissionRequestInt, seq, sysId, compId, p, outPacket);
        }
    }
}
