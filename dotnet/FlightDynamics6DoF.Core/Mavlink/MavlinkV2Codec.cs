// Licensed under the Apache License, Version 2.0.
// Clean-room zero-dependency MAVLink v2 wire protocol implementation (MIT wire specification).
using System;
using System.Buffers.Binary;
using System.Text;
using FlightDynamics6DoF.Core.Dynamics;
using FlightDynamics6DoF.Core.Math;
using FlightDynamics6DoF.Core.Sensors;

namespace FlightDynamics6DoF.Core.Mavlink
{
    public static class MavlinkMessageIds
    {
        public const uint Heartbeat = 0;
        public const uint SysStatus = 1;
        public const uint ParamRequestList = 21;
        public const uint ParamValue = 22;
        public const uint ParamSet = 23;
        public const uint Attitude = 30;
        public const uint LocalPositionNed = 32;
        public const uint GlobalPositionInt = 33;
        public const uint MissionCount = 44;
        public const uint MissionAck = 47;
        public const uint MissionRequestInt = 51;
        public const uint MissionItemInt = 73;
        public const uint VfrHud = 74;
        public const uint CommandLong = 76;
        public const uint CommandAck = 77;
        public const uint AttitudeTarget = 83;
        public const uint PositionTargetLocalNed = 85;
        public const uint HilActuatorControls = 93;
        public const uint HilSensor = 107;
        public const uint HilGps = 113;
        public const uint HilStateQuaternion = 115;
    }

    public struct MavlinkV2Frame
    {
        public byte Sequence;
        public byte SystemId;
        public byte ComponentId;
        public uint MessageId;
        public byte[] Payload;
        public int PayloadLength;
    }

    /// <summary>
    /// Zero-dependency MAVLink v2 serializer, X.25 CRC-16-CCITT calculator, and streaming byte parser
    /// for PX4 / ArduPilot SITL Lockstep HIL (TCP 4560 / UDP) and GCS telemetry.
    /// </summary>
    public static class MavlinkV2Codec
    {
        public const byte StxV2 = 0xFD;

        /// <summary>
        /// Official MAVLink CRC_EXTRA bytes per message ID.
        /// </summary>
        public static byte GetCrcExtra(uint msgId) => msgId switch
        {
            MavlinkMessageIds.Heartbeat => 50,
            MavlinkMessageIds.SysStatus => 124,
            MavlinkMessageIds.ParamRequestList => 159,
            MavlinkMessageIds.ParamValue => 220,
            MavlinkMessageIds.ParamSet => 168,
            MavlinkMessageIds.Attitude => 39,
            MavlinkMessageIds.LocalPositionNed => 185,
            MavlinkMessageIds.GlobalPositionInt => 104,
            MavlinkMessageIds.MissionCount => 221,
            MavlinkMessageIds.MissionAck => 153,
            MavlinkMessageIds.MissionRequestInt => 196,
            MavlinkMessageIds.MissionItemInt => 38,
            MavlinkMessageIds.VfrHud => 20,
            MavlinkMessageIds.CommandLong => 152,
            MavlinkMessageIds.CommandAck => 143,
            MavlinkMessageIds.AttitudeTarget => 22,
            MavlinkMessageIds.PositionTargetLocalNed => 140,
            MavlinkMessageIds.HilActuatorControls => 47,
            MavlinkMessageIds.HilSensor => 108,
            MavlinkMessageIds.HilGps => 124,
            MavlinkMessageIds.HilStateQuaternion => 4,
            _ => 0
        };

        public static ushort ComputeCrcX25(ReadOnlySpan<byte> headerAndPayload, byte crcExtra)
        {
            ushort crc = 0xFFFF;
            for (int i = 0; i < headerAndPayload.Length; i++)
                crc = AccumulateCrcX25(headerAndPayload[i], crc);
            crc = AccumulateCrcX25(crcExtra, crc);
            return crc;
        }

        private static ushort AccumulateCrcX25(byte b, ushort crc)
        {
            byte ch = (byte)(b ^ (byte)(crc & 0x00FF));
            ch = (byte)(ch ^ (ch << 4));
            return (ushort)((crc >> 8) ^ (ch << 8) ^ (ch << 3) ^ (ch >> 4));
        }

        /// <summary>
        /// Frames a MAVLink v2 packet into <paramref name="outBuffer"/> and returns the total wire length in bytes.
        /// </summary>
        public static int EncodeFrame(
            uint msgId,
            byte seq,
            byte sysId,
            byte compId,
            ReadOnlySpan<byte> payload,
            Span<byte> outBuffer)
        {
            int trimmedLen = payload.Length;
            while (trimmedLen > 1 && payload[trimmedLen - 1] == 0)
                trimmedLen--;

            outBuffer[0] = StxV2;
            outBuffer[1] = (byte)trimmedLen;
            outBuffer[2] = 0;
            outBuffer[3] = 0;
            outBuffer[4] = seq;
            outBuffer[5] = sysId;
            outBuffer[6] = compId;
            outBuffer[7] = (byte)(msgId & 0xFF);
            outBuffer[8] = (byte)((msgId >> 8) & 0xFF);
            outBuffer[9] = (byte)((msgId >> 16) & 0xFF);

            payload.Slice(0, trimmedLen).CopyTo(outBuffer.Slice(10, trimmedLen));

            ushort crc = ComputeCrcX25(outBuffer.Slice(1, 9 + trimmedLen), GetCrcExtra(msgId));
            outBuffer[10 + trimmedLen] = (byte)(crc & 0xFF);
            outBuffer[11 + trimmedLen] = (byte)((crc >> 8) & 0xFF);

            return 12 + trimmedLen;
        }

        public static int PackHeartbeat(byte seq, byte sysId, byte compId, byte mavType, byte autopilot, byte baseMode, uint customMode, byte systemStatus, Span<byte> outPacket)
        {
            Span<byte> p = stackalloc byte[9];
            BinaryPrimitives.WriteUInt32LittleEndian(p.Slice(0, 4), customMode);
            p[4] = mavType;
            p[5] = autopilot;
            p[6] = baseMode;
            p[7] = systemStatus;
            p[8] = 3;
            return EncodeFrame(MavlinkMessageIds.Heartbeat, seq, sysId, compId, p, outPacket);
        }

        public static int PackHilSensor(byte seq, byte sysId, byte compId, VirtualSensorReadings s, Span<byte> outPacket)
        {
            Span<byte> p = stackalloc byte[64];
            p.Clear();
            BinaryPrimitives.WriteUInt64LittleEndian(p.Slice(0, 8), s.TimestampMicroseconds);
            WriteSingle(p, 8, (float)s.ImuAccelBodyMps2.X);
            WriteSingle(p, 12, (float)s.ImuAccelBodyMps2.Y);
            WriteSingle(p, 16, (float)s.ImuAccelBodyMps2.Z);
            WriteSingle(p, 20, (float)s.ImuGyroBodyRadps.X);
            WriteSingle(p, 24, (float)s.ImuGyroBodyRadps.Y);
            WriteSingle(p, 28, (float)s.ImuGyroBodyRadps.Z);
            WriteSingle(p, 32, (float)s.MagnetometerBodyGauss.X);
            WriteSingle(p, 36, (float)s.MagnetometerBodyGauss.Y);
            WriteSingle(p, 40, (float)s.MagnetometerBodyGauss.Z);
            WriteSingle(p, 44, (float)s.BaroPressureHpa);
            WriteSingle(p, 48, (float)s.DifferentialPressureHpa);
            WriteSingle(p, 52, (float)s.BaroAltitudeMsl);
            WriteSingle(p, 56, (float)s.TemperatureCelsius);
            BinaryPrimitives.WriteUInt32LittleEndian(p.Slice(60, 4), 0x1FFFu);
            return EncodeFrame(MavlinkMessageIds.HilSensor, seq, sysId, compId, p, outPacket);
        }

        public static int PackHilGps(byte seq, byte sysId, byte compId, VirtualSensorReadings s, Span<byte> outPacket)
        {
            Span<byte> p = stackalloc byte[36];
            p.Clear();
            BinaryPrimitives.WriteUInt64LittleEndian(p.Slice(0, 8), s.TimestampMicroseconds);
            BinaryPrimitives.WriteInt32LittleEndian(p.Slice(8, 4), (int)System.Math.Round(s.GpsLatitudeDeg * 1e7));
            BinaryPrimitives.WriteInt32LittleEndian(p.Slice(12, 4), (int)System.Math.Round(s.GpsLongitudeDeg * 1e7));
            BinaryPrimitives.WriteInt32LittleEndian(p.Slice(16, 4), (int)System.Math.Round(s.GpsAltitudeMsl * 1000.0));
            BinaryPrimitives.WriteUInt16LittleEndian(p.Slice(20, 2), 35);
            BinaryPrimitives.WriteUInt16LittleEndian(p.Slice(22, 2), 60);
            BinaryPrimitives.WriteUInt16LittleEndian(p.Slice(24, 2), (ushort)System.Math.Round(s.GpsGroundSpeedMps * 100.0));
            BinaryPrimitives.WriteInt16LittleEndian(p.Slice(26, 2), (short)System.Math.Round(s.GpsVelocityNedMps.X * 100.0));
            BinaryPrimitives.WriteInt16LittleEndian(p.Slice(28, 2), (short)System.Math.Round(s.GpsVelocityNedMps.Y * 100.0));
            BinaryPrimitives.WriteInt16LittleEndian(p.Slice(30, 2), (short)System.Math.Round(s.GpsVelocityNedMps.Z * 100.0));
            BinaryPrimitives.WriteUInt16LittleEndian(p.Slice(32, 2), (ushort)System.Math.Round(s.GpsCourseOverGroundDeg * 100.0));
            p[34] = (byte)s.GpsFixType;
            p[35] = (byte)s.GpsSatellitesVisible;
            return EncodeFrame(MavlinkMessageIds.HilGps, seq, sysId, compId, p, outPacket);
        }

        /// <summary>
        /// Packs HIL_STATE_QUATERNION (#115, 64 bytes) providing PX4 SITL with ground-truth attitude, rates, position, and velocity.
        /// </summary>
        public static int PackHilStateQuaternion(
            byte seq,
            byte sysId,
            byte compId,
            ulong timeUsec,
            SixDofState state,
            double latDeg,
            double lonDeg,
            double altMslMeters,
            Span<byte> outPacket)
        {
            Span<byte> p = stackalloc byte[64];
            p.Clear();
            BinaryPrimitives.WriteUInt64LittleEndian(p.Slice(0, 8), timeUsec);
            WriteSingle(p, 8, (float)state.AttitudeBodyToNed.W);
            WriteSingle(p, 12, (float)state.AttitudeBodyToNed.X);
            WriteSingle(p, 16, (float)state.AttitudeBodyToNed.Y);
            WriteSingle(p, 20, (float)state.AttitudeBodyToNed.Z);
            WriteSingle(p, 24, (float)state.AngularVelocityBodyFrd.X);
            WriteSingle(p, 28, (float)state.AngularVelocityBodyFrd.Y);
            WriteSingle(p, 32, (float)state.AngularVelocityBodyFrd.Z);
            BinaryPrimitives.WriteInt32LittleEndian(p.Slice(36, 4), (int)System.Math.Round(latDeg * 1e7));
            BinaryPrimitives.WriteInt32LittleEndian(p.Slice(40, 4), (int)System.Math.Round(lonDeg * 1e7));
            BinaryPrimitives.WriteInt32LittleEndian(p.Slice(44, 4), (int)System.Math.Round(altMslMeters * 1000.0));
            BinaryPrimitives.WriteInt16LittleEndian(p.Slice(48, 2), (short)System.Math.Round(state.VelocityNed.X * 100.0));
            BinaryPrimitives.WriteInt16LittleEndian(p.Slice(50, 2), (short)System.Math.Round(state.VelocityNed.Y * 100.0));
            BinaryPrimitives.WriteInt16LittleEndian(p.Slice(52, 2), (short)System.Math.Round(state.VelocityNed.Z * 100.0));
            BinaryPrimitives.WriteUInt16LittleEndian(p.Slice(54, 2), (ushort)System.Math.Round(state.AirspeedMps * 100.0));
            BinaryPrimitives.WriteUInt16LittleEndian(p.Slice(56, 2), (ushort)System.Math.Round(state.AirspeedMps * 100.0));
            BinaryPrimitives.WriteInt16LittleEndian(p.Slice(58, 2), (short)System.Math.Round(state.SpecificForceBodyFrd.X * 1000.0 / 9.80665));
            BinaryPrimitives.WriteInt16LittleEndian(p.Slice(60, 2), (short)System.Math.Round(state.SpecificForceBodyFrd.Y * 1000.0 / 9.80665));
            BinaryPrimitives.WriteInt16LittleEndian(p.Slice(62, 2), (short)System.Math.Round(state.SpecificForceBodyFrd.Z * 1000.0 / 9.80665));
            return EncodeFrame(MavlinkMessageIds.HilStateQuaternion, seq, sysId, compId, p, outPacket);
        }

        public static int PackAttitude(byte seq, byte sysId, byte compId, uint timeBootMs, Quatd qBodyToNed, Vec3d omegaFrd, Span<byte> outPacket)
        {
            Span<byte> p = stackalloc byte[28];
            Vec3d euler = qBodyToNed.ToEulerZyx();
            BinaryPrimitives.WriteUInt32LittleEndian(p.Slice(0, 4), timeBootMs);
            WriteSingle(p, 4, (float)euler.X);
            WriteSingle(p, 8, (float)euler.Y);
            WriteSingle(p, 12, (float)euler.Z);
            WriteSingle(p, 16, (float)omegaFrd.X);
            WriteSingle(p, 20, (float)omegaFrd.Y);
            WriteSingle(p, 24, (float)omegaFrd.Z);
            return EncodeFrame(MavlinkMessageIds.Attitude, seq, sysId, compId, p, outPacket);
        }

        public static int PackGlobalPositionInt(
            byte seq, byte sysId, byte compId, uint timeBootMs,
            double latDeg, double lonDeg, double altMslM, double relativeAltM,
            Vec3d velNedMps, double headingDeg, Span<byte> outPacket)
        {
            Span<byte> p = stackalloc byte[28];
            BinaryPrimitives.WriteUInt32LittleEndian(p.Slice(0, 4), timeBootMs);
            BinaryPrimitives.WriteInt32LittleEndian(p.Slice(4, 4), (int)System.Math.Round(latDeg * 1e7));
            BinaryPrimitives.WriteInt32LittleEndian(p.Slice(8, 4), (int)System.Math.Round(lonDeg * 1e7));
            BinaryPrimitives.WriteInt32LittleEndian(p.Slice(12, 4), (int)System.Math.Round(altMslM * 1000.0));
            BinaryPrimitives.WriteInt32LittleEndian(p.Slice(16, 4), (int)System.Math.Round(relativeAltM * 1000.0));
            BinaryPrimitives.WriteInt16LittleEndian(p.Slice(20, 2), (short)System.Math.Round(velNedMps.X * 100.0));
            BinaryPrimitives.WriteInt16LittleEndian(p.Slice(22, 2), (short)System.Math.Round(velNedMps.Y * 100.0));
            BinaryPrimitives.WriteInt16LittleEndian(p.Slice(24, 2), (short)System.Math.Round(velNedMps.Z * 100.0));
            ushort hdgCdeg = (ushort)(((headingDeg % 360.0 + 360.0) % 360.0) * 100.0);
            BinaryPrimitives.WriteUInt16LittleEndian(p.Slice(26, 2), hdgCdeg);
            return EncodeFrame(MavlinkMessageIds.GlobalPositionInt, seq, sysId, compId, p, outPacket);
        }

        public static int PackHilActuatorControls(
            byte seq, byte sysId, byte compId, ulong timeUsec, ReadOnlySpan<double> controls16, byte mode, ulong flags, Span<byte> outPacket)
        {
            Span<byte> p = stackalloc byte[81];
            p.Clear();
            BinaryPrimitives.WriteUInt64LittleEndian(p.Slice(0, 8), timeUsec);
            BinaryPrimitives.WriteUInt64LittleEndian(p.Slice(8, 8), flags);
            for (int i = 0; i < 16; i++)
            {
                float val = i < controls16.Length ? (float)controls16[i] : 0.0f;
                WriteSingle(p, 16 + i * 4, val);
            }
            p[80] = mode;
            return EncodeFrame(MavlinkMessageIds.HilActuatorControls, seq, sysId, compId, p, outPacket);
        }

        public static void UnpackHilActuatorControls(ReadOnlySpan<byte> payload, Span<double> outControls16, out ulong timeUsec, out byte mode)
        {
            Span<byte> full = stackalloc byte[81];
            full.Clear();
            payload.Slice(0, System.Math.Min(payload.Length, 81)).CopyTo(full);

            timeUsec = BinaryPrimitives.ReadUInt64LittleEndian(full.Slice(0, 8));
            for (int i = 0; i < 16 && i < outControls16.Length; i++)
            {
                float c = ReadSingle(full, 16 + i * 4);
                outControls16[i] = float.IsNaN(c) ? 0.0 : c;
            }
            mode = full[80];
        }

        public static void UnpackHilActuatorControls(ReadOnlySpan<byte> payload, Span<double> outControls16, out ulong timeUsec) =>
            UnpackHilActuatorControls(payload, outControls16, out timeUsec, out _);

        public static void UnpackAttitudeTarget(
            ReadOnlySpan<byte> payload,
            out Quatd attitudeBodyToNed,
            out float thrust,
            out byte typeMask)
        {
            Span<byte> full = stackalloc byte[37];
            full.Clear();
            payload.Slice(0, System.Math.Min(payload.Length, 37)).CopyTo(full);

            float qw = ReadSingle(full, 4);
            float qx = ReadSingle(full, 8);
            float qy = ReadSingle(full, 12);
            float qz = ReadSingle(full, 16);
            thrust = ReadSingle(full, 32);
            typeMask = full[36];
            attitudeBodyToNed = new Quatd(qw, qx, qy, qz).Normalized();
        }

        public static void UnpackPositionTargetLocalNed(
            ReadOnlySpan<byte> payload,
            out double posN,
            out double posE,
            out double posD,
            out double velN,
            out double velE,
            out double velD,
            out double yawRad,
            out ushort typeMask)
        {
            Span<byte> full = stackalloc byte[51];
            full.Clear();
            payload.Slice(0, System.Math.Min(payload.Length, 51)).CopyTo(full);

            posN = ReadSingle(full, 4);
            posE = ReadSingle(full, 8);
            posD = ReadSingle(full, 12);
            velN = ReadSingle(full, 16);
            velE = ReadSingle(full, 20);
            velD = ReadSingle(full, 24);
            yawRad = ReadSingle(full, 40);
            typeMask = BinaryPrimitives.ReadUInt16LittleEndian(full.Slice(48, 2));
        }

        public static void UnpackCommandLong(
            ReadOnlySpan<byte> payload,
            out ushort command,
            out float param1,
            out float param2,
            out float param5,
            out float param6,
            out float param7)
        {
            Span<byte> full = stackalloc byte[33];
            full.Clear();
            payload.Slice(0, System.Math.Min(payload.Length, 33)).CopyTo(full);

            param1 = ReadSingle(full, 0);
            param2 = ReadSingle(full, 4);
            param5 = ReadSingle(full, 16);
            param6 = ReadSingle(full, 20);
            param7 = ReadSingle(full, 24);
            command = BinaryPrimitives.ReadUInt16LittleEndian(full.Slice(28, 2));
        }

        public static int PackCommandAck(byte seq, byte sysId, byte compId, ushort command, byte result, Span<byte> outPacket)
        {
            Span<byte> p = stackalloc byte[10];
            p.Clear();
            BinaryPrimitives.WriteUInt16LittleEndian(p.Slice(0, 2), command);
            p[2] = result;
            return EncodeFrame(MavlinkMessageIds.CommandAck, seq, sysId, compId, p, outPacket);
        }

        public static int PackParamValue(byte seq, byte sysId, byte compId, string paramId, float paramValue, ushort paramCount, ushort paramIndex, Span<byte> outPacket)
        {
            Span<byte> p = stackalloc byte[25];
            p.Clear();
            WriteSingle(p, 0, paramValue);
            BinaryPrimitives.WriteUInt16LittleEndian(p.Slice(4, 2), paramCount);
            BinaryPrimitives.WriteUInt16LittleEndian(p.Slice(6, 2), paramIndex);
            byte[] ascii = Encoding.ASCII.GetBytes(paramId);
            int copyLen = System.Math.Min(16, ascii.Length);
            ascii.AsSpan(0, copyLen).CopyTo(p.Slice(8, 16));
            p[24] = 9;
            return EncodeFrame(MavlinkMessageIds.ParamValue, seq, sysId, compId, p, outPacket);
        }

        public static void UnpackParamSet(ReadOnlySpan<byte> payload, out string paramId, out float paramValue)
        {
            Span<byte> full = stackalloc byte[23];
            full.Clear();
            payload.Slice(0, System.Math.Min(payload.Length, 23)).CopyTo(full);

            paramValue = ReadSingle(full, 0);
            int strLen = 0;
            while (strLen < 16 && full[6 + strLen] != 0) strLen++;
            paramId = Encoding.ASCII.GetString(full.Slice(6, strLen));
        }

        public static void WriteSingle(Span<byte> buf, int offset, float value)
        {
            int bits = BitConverter.SingleToInt32Bits(value);
            BinaryPrimitives.WriteInt32LittleEndian(buf.Slice(offset, 4), bits);
        }

        public static float ReadSingle(ReadOnlySpan<byte> buf, int offset)
        {
            int bits = BinaryPrimitives.ReadInt32LittleEndian(buf.Slice(offset, 4));
            return BitConverter.Int32BitsToSingle(bits);
        }
    }

    /// <summary>
    /// Streaming byte parser for MAVLink v2 frames.
    /// </summary>
    public class MavlinkV2StreamParser
    {
        private readonly byte[] _buffer = new byte[280];
        private readonly byte[] _rxPayload = new byte[255];
        private int _index = 0;
        private int _expectedLength = 0;

        public bool PushByte(byte b, out MavlinkV2Frame frame)
        {
            frame = default;

            if (_index == 0)
            {
                if (b == MavlinkV2Codec.StxV2)
                {
                    _buffer[_index++] = b;
                }
                return false;
            }

            _buffer[_index++] = b;

            if (_index == 2)
            {
                int payloadLen = _buffer[1];
                _expectedLength = 12 + payloadLen;
                if (_expectedLength > _buffer.Length)
                {
                    _index = 0;
                    return false;
                }
            }
            else if (_index >= 12 && _index == _expectedLength)
            {
                int payloadLen = _buffer[1];
                uint msgId = (uint)(_buffer[7] | (_buffer[8] << 8) | (_buffer[9] << 16));
                ushort receivedCrc = BinaryPrimitives.ReadUInt16LittleEndian(_buffer.AsSpan(10 + payloadLen, 2));
                ushort computedCrc = MavlinkV2Codec.ComputeCrcX25(_buffer.AsSpan(1, 9 + payloadLen), MavlinkV2Codec.GetCrcExtra(msgId));

                _index = 0;
                if (receivedCrc == computedCrc)
                {
                    Buffer.BlockCopy(_buffer, 10, _rxPayload, 0, payloadLen);
                    frame = new MavlinkV2Frame
                    {
                        Sequence = _buffer[4],
                        SystemId = _buffer[5],
                        ComponentId = _buffer[6],
                        MessageId = msgId,
                        Payload = _rxPayload,
                        PayloadLength = payloadLen
                    };
                    return true;
                }
            }

            return false;
        }
    }
}
