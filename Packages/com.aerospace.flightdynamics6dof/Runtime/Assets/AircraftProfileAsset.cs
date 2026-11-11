// Licensed under the Apache License, Version 2.0.
using System;
using System.IO;
using UnityEngine;
using FlightDynamics6DoF.Core.Aerodynamics;
using FlightDynamics6DoF.Core.Gnc;
using FlightDynamics6DoF.Core.Specs;

namespace FlightDynamics6DoF.Unity
{
    /// <summary>
    /// Unity ScriptableObject container holding both Stage 1 Input Configuration (<see cref="AircraftConfigSpec"/>)
    /// and Stage 1 Pre-computed Aerodynamic/Stability/Trim Database (<see cref="BakedAeroDatabase"/>).
    /// Supports JSON import/export and PX4 v1.14+ Control Allocation script generation.
    /// </summary>
    [CreateAssetMenu(
        fileName = "NewAircraftProfile",
        menuName = "FlightDynamics 6-DOF/Aircraft Profile Asset",
        order = 10)]
    public class AircraftProfileAsset : ScriptableObject
    {
        [Header("Stage 1: Aircraft Geometry, Propulsion & Aerodynamic Specification")]
        public AircraftConfigSpec Config = AircraftConfigSpec.CreateStandardQuadcopterX();

        [Header("Stage 1 Output: Baked Aerodynamic, Stability & Trim Database")]
        public BakedAeroDatabase BakedDatabase = new BakedAeroDatabase();

        /// <summary>
        /// Executes Stage 1 (Pre-Simulation Aerodynamic, BEMT, 3D VLM, Slender Body &amp; Stability Analysis)
        /// and stores the resulting <see cref="BakedAeroDatabase"/> into this asset.
        /// </summary>
        [ContextMenu("Run Stage 1 Aerodynamic Analysis & Bake")]
        public BakedAeroDatabase AnalyzeAndBake()
        {
            if (Config == null)
                Config = AircraftConfigSpec.CreateStandardQuadcopterX();

            BakedDatabase = StabilityAndTrimAnalyzer.AnalyzeAndBake(Config);
            return BakedDatabase;
        }

        public void LoadPreset(VehicleAirframeCategory presetCategory)
        {
            switch (presetCategory)
            {
                case VehicleAirframeCategory.Multirotor:
                    Config = AircraftConfigSpec.CreateStandardQuadcopterX();
                    break;
                case VehicleAirframeCategory.FixedWing:
                    Config = AircraftConfigSpec.CreateFixedWingUav();
                    break;
                case VehicleAirframeCategory.HybridQuadPlaneVtol:
                case VehicleAirframeCategory.TiltRotorVtol:
                    Config = AircraftConfigSpec.CreateQuadPlaneVtolMillie();
                    break;
            }
            AnalyzeAndBake();
        }

        public string ExportConfigToJson(bool prettyPrint = true) =>
            JsonUtility.ToJson(Config, prettyPrint);

        public void ImportConfigFromJson(string json)
        {
            if (string.IsNullOrWhiteSpace(json)) return;
            Config = JsonUtility.FromJson<AircraftConfigSpec>(json);
            AnalyzeAndBake();
        }

        public string GeneratePx4ParameterScript()
        {
            if (BakedDatabase == null || BakedDatabase.RotorLumpedKt == null || BakedDatabase.RotorLumpedKt.Length == 0)
                AnalyzeAndBake();
            return WrenchControlAllocator.GeneratePx4AirframeParameterScript(Config, BakedDatabase!);
        }
    }
}
